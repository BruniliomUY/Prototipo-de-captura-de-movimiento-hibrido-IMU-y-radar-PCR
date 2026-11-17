# IMU de la Pico W (CircuitPython).
#
# Contrato: crear_imu() devuelve un objeto con
#   TIPO      : str corto que se manda a la Zero al conectar ("bno085", "sim")
#   leer()    : dict con accel_x, accel_y, accel_z (m/s²), roll, pitch, yaw (grados)
#               e imu_ok (1/0), las mismas claves que usa pi_master.py.
#               Ante un error NO devuelve ceros: repite el último valor con imu_ok=0.
#
# ImuBno085 replica la lógica de la clase Imu de pi_master.py: descarta cuaterniones
# corruptos, reinicia el sensor ante OSError, errores seguidos o datos congelados, y si
# el sensor no está (o no arranca) sigue mandando muestras con imu_ok=0 y reintenta.
#
# Los tiempos se manejan en ms enteros (time.monotonic_ns): time.monotonic() es float de
# baja precisión en CircuitPython y pierde resolución con las horas de uso.

import math
import time

import board
import busio

import config

IMU_NORMA_TOL = 0.02        # tolerancia de |q| respecto de 1 (cuaternión válido)
IMU_ERRORES_MAX = 3         # errores seguidos (que no sean OSError) antes de reiniciar
IMU_CONGELADO_MS = 5000     # accel + cuaternión idénticos este tiempo => IMU congelado
REINTENTO_INIT_MS = 2000    # si el sensor no arranca, cada cuánto reintentar

# --- IMU simulada (config.IMU = "sim"): una sinusoide constante, sin ruido ni saltos ---
#   canal(t) = centro + amplitud * sin(2*pi*SIM_HZ*t + fase)
# Todos los canales usan la misma frecuencia; cambian la amplitud y la fase para poder
# distinguirlos en un gráfico. Con SIM_HZ = 1 y 20 muestras/s hay 20 puntos por ciclo.
SIM_HZ = 1.0                # ciclos por segundo
SIM_CANALES = {             # (centro, amplitud, fase en grados)
    "accel_x": (0.0, 2.0, 0),      # m/s²
    "accel_y": (0.0, 2.0, 90),
    "accel_z": (9.81, 2.0, 0),
    "roll": (0.0, 30.0, 0),        # grados
    "pitch": (0.0, 15.0, 90),
    "yaw": (0.0, 45.0, 0),
}


def ms():
    return time.monotonic_ns() // 1_000_000


def quat_to_euler(qi, qj, qk, qr):
    w, x, y, z = qr, qi, qj, qk
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.degrees(math.asin(sinp))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


def _muestra(accel, euler, ok):
    return dict(
        accel_x=round(accel[0], 4), accel_y=round(accel[1], 4), accel_z=round(accel[2], 4),
        roll=round(euler[0], 3), pitch=round(euler[1], 3), yaw=round(euler[2], 3),
        imu_ok=1 if ok else 0,
    )


class ImuBno085:
    TIPO = "bno085"

    def __init__(self):
        self.reset = None
        if config.BNO_RESET_PIN is not None:
            import digitalio
            self.reset = digitalio.DigitalInOut(getattr(board, config.BNO_RESET_PIN))
        self.i2c = None
        self.bno = None
        self.ultimo_accel = (0.0, 0.0, 0.0)
        self.ultimo_euler = (0.0, 0.0, 0.0)
        self.reinicios = 0
        self._iniciar()

    def _liberar(self):
        if self.i2c is not None:
            try:
                self.i2c.deinit()
            except Exception:
                pass
        self.i2c = None
        self.bno = None

    def _iniciar(self):
        from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_ROTATION_VECTOR
        from adafruit_bno08x.i2c import BNO08X_I2C

        self._liberar()
        try:
            self.i2c = busio.I2C(getattr(board, config.I2C_SCL), getattr(board, config.I2C_SDA),
                                 frequency=config.I2C_FREQ)
            bno = BNO08X_I2C(self.i2c, reset=self.reset, address=config.BNO_ADDR)
            bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            time.sleep(0.15)
            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            time.sleep(0.3)
            self.bno = bno
            print("✓ IMU BNO085 lista")
        except Exception as e:
            print("IMU: no arrancó (%s: %s)" % (type(e).__name__, e))
            self._liberar()
        self.errores_seguidos = 0
        self.firma = None
        self.t_cambio = ms()
        self.t_intento = ms()

    def reiniciar(self, motivo):
        self.reinicios += 1
        print("[IMU] Reiniciando #%d (%s)" % (self.reinicios, motivo))
        self._iniciar()

    def _invalida(self):
        return _muestra(self.ultimo_accel, self.ultimo_euler, False)

    def leer(self):
        if self.bno is None:
            if ms() - self.t_intento >= REINTENTO_INIT_MS:
                self._iniciar()
            if self.bno is None:
                return self._invalida()

        try:
            accel = tuple(self.bno.acceleration)
            q = tuple(self.bno.quaternion)  # (i, j, k, real)
            # La librería a veces deja pasar paquetes corruptos sin excepción:
            # un cuaternión válido tiene |q| = 1.
            norma = sum(c * c for c in q) ** 0.5
            if not abs(norma - 1.0) <= IMU_NORMA_TOL:
                raise ValueError("cuaternión inválido (|q|=%.3f)" % norma)
            euler = quat_to_euler(*q)
        except Exception as e:
            self.errores_seguidos += 1
            print("[IMU ERROR] %s: %s" % (type(e).__name__, e))
            # OSError = el sensor dejó de contestar en el bus: reinicio inmediato.
            if isinstance(e, OSError):
                self.reiniciar("OSError")
            elif self.errores_seguidos >= IMU_ERRORES_MAX:
                self.reiniciar("%d errores seguidos" % self.errores_seguidos)
            return self._invalida()

        self.errores_seguidos = 0
        self.ultimo_accel, self.ultimo_euler = accel, euler

        # Congelado: el sensor dejó de mandar reportes y la librería repite el último valor.
        firma = (accel, q)
        ahora = ms()
        if firma != self.firma:
            self.firma = firma
            self.t_cambio = ahora
        elif ahora - self.t_cambio > IMU_CONGELADO_MS:
            self.reiniciar("sin datos nuevos (IMU congelado)")
            return self._invalida()

        return _muestra(accel, euler, True)


class ImuSimulada:
    """Sinusoide constante (ver SIM_HZ y SIM_CANALES) para probar red, buffer, sincronización
    y CSV sin sensor. Siempre imu_ok=1."""

    TIPO = "sim"
    reinicios = 0

    def __init__(self):
        self.t0 = ms()
        # El ángulo se calcula con enteros (ms dentro del ciclo): los float de CircuitPython son
        # de 32 bits y 2*pi*f*t pierde precisión con las horas de uso.
        self.periodo_ms = max(1, int(round(1000.0 / SIM_HZ)))

    def _canal(self, nombre, ang):
        centro, amplitud, fase = SIM_CANALES[nombre]
        return centro + amplitud * math.sin(ang + math.radians(fase))

    def leer(self):
        ang = 2 * math.pi * ((ms() - self.t0) % self.periodo_ms) / self.periodo_ms
        return _muestra(
            (self._canal("accel_x", ang), self._canal("accel_y", ang), self._canal("accel_z", ang)),
            (self._canal("roll", ang), self._canal("pitch", ang), self._canal("yaw", ang)),
            True,
        )


def crear_imu():
    if config.IMU == "bno085":
        return ImuBno085()
    if config.IMU == "sim":
        return ImuSimulada()
    raise ValueError("IMU desconocida en config.py: %r" % config.IMU)
