#!/usr/bin/env python3
"""
Corre en la Raspberry Pi.
Lee IMU (BNO085) + Radar (Acconeer XM125), guarda CSV local como respaldo,
y transmite cada muestra por TCP (JSON, una línea por muestra) a quien se conecte.

No requiere matplotlib ni entorno gráfico en el Pi.

I2C del BNO085: usa clock stretching y la Pi lo maneja mal (KeyError / OSError al
leer). La velocidad real del bus se fija en /boot/firmware/config.txt (antes de
Bookworm: /boot/config.txt), no desde Python (Blinka ignora el parámetro frequency):
    dtparam=i2c_arm_baudrate=400000
y hay que reiniciar la Pi para que tome efecto.

Cambios de esta versión:
  - IMU: IMU_ERRORES_MAX = 3 y IMU_CONGELADO_S = 5.0 (menos falsos positivos con el
    sensor quieto). Al final se imprime cuántos reinicios de IMU y reconexiones de
    radar hubo, y la línea de estado muestra si la IMU está ok.
  - Radar: peaksorting_method = CLOSEST (con aluminio, STRONGEST devolvía ecos
    fantasma a 2x y 3x de la distancia real), PROFILE_2, threshold_sensitivity = 0.7,
    start_m = 0.03 (límite con cancelación de fuga) y update_rate = 20.
    CLOSE_RANGE = False pasa a modo robusto: sin cancelación de fuga, start_m = 0.07.
  - Correcciones: NameError por PORT (ahora PUERTO, usado también en bind),
    CLOSE_RANGE duplicado, init_radar usa RADAR_BAUDRATE / RADAR_FLOW_CONTROL,
    y except sin tipo reemplazados por except Exception.

MODO MASTER / ESCLAVA (esta versión)
  El script queda corriendo y espera órdenes por el puerto 5004 (ver clase Control):
      ON      la Pico prepara su IMU; esta master inicia IMU + radar (calibración);
              cuando las dos están listas se fija T0 y las dos miden desde ahí.
      OFF     las dos paran; se devuelve medicion_<sid>.csv con los datos de ambas.
      STATUS  estado actual.   CSV  vuelve a pedir el último CSV.
  La Pico (esclava) mide solo la IMU y manda sus muestras por el puerto 5006
  (pico_receptor.py). Archivos por sesión:
      datos_<sid>.csv     crudo de la master (igual que siempre)
      pico_<sid>.csv      crudo de la Pico, ya llevado al reloj de la master
      medicion_<sid>.csv  los dos unidos por tiempo (es el que devuelve OFF)
  No cambió: Imu, init_imu, init_radar, leer_radar, CLOSE_RANGE, RADAR_CONFIG, la
  protección anti-IMU ni la reconexión del radar. En el loop solo se agregó que, hasta T0,
  se lee todo pero no se guarda.
"""

import time
import math
import csv
import json
import socket
import signal
import sys
import logging
import os
import queue
import threading
import bisect
from datetime import datetime

logging.getLogger("adafruit_bno08x").setLevel(logging.CRITICAL)
logging.getLogger().setLevel(logging.CRITICAL)

import board
import busio
import serial
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

from acconeer.exptool import a121
from acconeer.exptool.a121.algo.distance import (
    Detector,
    DetectorConfig,
    PeakSortingMethod,
    ReflectorShape,
    ThresholdMethod,
)

from pico_receptor import ReceptorPico

# El servidor escucha en 0.0.0.0 (todas las interfaces), así que no hace falta
# fijar ninguna IP: la PC se conecta por hostname (<hostname>.local) o por la IP
# que se imprime al arrancar.
PUERTO = 5005            # transmisión en vivo a la PC (opcional, como antes)
PUERTO_CONTROL = 5004    # ON / OFF / STATUS / CSV
# La Pico se conecta al puerto 5006 (PICO_PORT en pico_receptor.py)

# --- Sesiones ON / OFF ---
USAR_PICO = True            # False = medir solo con la master (sin esclava)
RECALIBRAR_CADA_ON = True   # True = cada ON calibra el radar de nuevo; False = reutiliza la
                            # calibración de la primera sesión (más rápido, pero solo si el
                            # montaje no se movió)
MARGEN_START_S = 1.0        # T0 = fin de la calibración + este margen (que le llegue 'start' a la Pico)
PICO_LISTO_TIMEOUT_S = 20.0 # máximo que ON espera el 'listo' de la Pico
PICO_FIN_TIMEOUT_S = 10.0   # máximo que OFF espera que la Pico vacíe sus pendientes
MERGE_TOL_S = 0.075         # una muestra de la Pico se pega a la de la master si están a <75 ms

# --- IMU: validación y recuperación automática ---
IMU_RESET_PIN = None    # p. ej. board.D17 si conectás el pin RST del BNO085 a ese GPIO
IMU_NORMA_TOL = 0.02    # tolerancia de |q| respecto de 1 (cuaternión válido)
IMU_ERRORES_MAX = 3     # errores seguidos (que no sean OSError) antes de reiniciar el IMU
IMU_CONGELADO_S = 5.0   # accel + cuaternión idénticos durante este tiempo => IMU congelado
                        # (con 2 s un sensor muy quieto puede dar falsos reinicios; validar
                        #  con la prueba de 2 minutos quieto: los reinicios deben ser 0)

# --- Radar: configuración del detector de distancia ---
# CLOSE_RANGE = True : mide desde ~3 cm (límite de Acconeer con cancelación de fuga), pero
#   - la calibración (que se repite en cada reconexión del radar) necesita espacio
#     libre frente al sensor, ya montado en su geometría final;
#   - cada reconexión puede tardar ~30 s a 115200 (ver el print del arranque).
# CLOSE_RANGE = False: sin cancelación de fuga; mide desde ~7 cm, la reconexión tarda ~3 s
#   y no exige frente libre al calibrar.
CLOSE_RANGE = True

RADAR_CONFIG = dict(
    start_m=0.03 if CLOSE_RANGE else 0.07,
    end_m=0.65,
    max_step_length=2,
    max_profile=a121.Profile.PROFILE_2,
    close_range_leakage_cancellation=CLOSE_RANGE,
    signal_quality=20.0,
    threshold_method=ThresholdMethod.CFAR,
    peaksorting_method=PeakSortingMethod.CLOSEST,
    reflector_shape=ReflectorShape.GENERIC,
    num_frames_in_recorded_threshold=10,
    fixed_threshold_value=100.0,
    fixed_strength_threshold_value=0.0,
    threshold_sensitivity=0.7,
    update_rate=20.0,
)

# UART del radar. Con close range, calibrate_detector() graba num_frames_in_recorded_threshold
# frames completos (~3.5 KB c/u => ~100 KB con 30 frames): un solo byte corrupto la aborta.
RADAR_PUERTO_SERIE = "/dev/serial0"
RADAR_BAUDRATE = 115200     # None = el cliente usa el max_baudrate del servidor (o 115200); 115200 fuerza la base
RADAR_FLOW_CONTROL = True   # RTS/CTS por hardware; probá False si solo tenés TX/RX/GND cableados


# -----------------------------------------------------------
class DummyFile(object):
    def write(self, x): pass
    def flush(self): pass


class SilenciarSalida:
    def __enter__(self):
        self._stdout = sys.stdout
        sys.stdout = DummyFile()

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout = self._stdout


def quat_to_euler(qi, qj, qk, qr):
    w, x, y, z = qr, qi, qj, qk
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.degrees(math.asin(sinp))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


# -----------------------------------------------------------
def crear_reset_imu():
    """Devuelve el pin de reset del BNO085 (o None si no está conectado)."""
    if IMU_RESET_PIN is None:
        return None
    import digitalio
    return digitalio.DigitalInOut(IMU_RESET_PIN)


def init_imu(reset=None, retries=3):
    """Devuelve (bno, i2c). Se guarda el bus para poder cerrarlo al reiniciar."""
    for intento in range(1, retries + 1):
        i2c = None
        try:
            print("Iniciando IMU BNO085 por I2C...")
            with SilenciarSalida():
                # La velocidad real del bus la fija config.txt (ver docstring).
                i2c = busio.I2C(board.SCL, board.SDA)
                time.sleep(0.5)
                bno = BNO08X_I2C(i2c, reset=reset)
                bno.enable_feature(BNO_REPORT_ACCELEROMETER)
                time.sleep(0.15)
                bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                time.sleep(0.3)
            print(f"✓ OK: IMU BNO085 lista (intento {intento})")
            return bno, i2c
        except Exception as e:
            print(f"  Intento {intento}/{retries} falló: {type(e).__name__}: {e}")
            if i2c is not None:
                try:
                    i2c.deinit()
                except Exception:
                    pass
            time.sleep(1.0)
    raise RuntimeError("No se pudo inicializar el IMU tras varios intentos")


class Imu:
    """
    Envuelve el BNO085:
      - descarta cuaterniones corruptos (|q| lejos de 1),
      - reinicia el sensor si el bus falla (OSError), si hay errores seguidos
        o si deja de mandar datos nuevos (congelado),
      - ante un error NO devuelve ceros: repite el último valor válido con ok=False.

    Nota para el análisis: los IMU_CONGELADO_S segundos previos a cada detección de
    congelamiento quedan registrados con imu_ok=1; descartarlos al procesar el CSV.
    """

    def __init__(self, reset=None):
        self.reset = reset
        self.bno, self.i2c = init_imu(reset)
        self.ultimo_accel = (0.0, 0.0, 0.0)
        self.ultimo_euler = (0.0, 0.0, 0.0)
        self.errores_seguidos = 0
        self.firma = None
        self.t_cambio = time.time()
        self.reinicios = 0

    def _muestra_invalida(self):
        # Para marcar con NaN en vez de repetir el último valor, devolver
        # (nan, nan, nan) acá (ojo: el JSON de Python escribe NaN, que no es JSON estricto).
        return dict(accel=self.ultimo_accel, euler=self.ultimo_euler, ok=False)

    def reiniciar(self, motivo):
        self.reinicios += 1
        print(f"\n[IMU] Reiniciando #{self.reinicios} ({motivo})...", file=sys.stderr)
        try:
            self.i2c.deinit()
        except Exception:
            pass
        try:
            self.bno, self.i2c = init_imu(self.reset)
            time.sleep(0.5)  # dejar que lleguen los primeros reportes
        except Exception as e:
            print(f"[IMU] Falló el reinicio: {e}", file=sys.stderr)
            time.sleep(2.0)  # evitar reintentar en loop cerrado si sigue fallando
        self.errores_seguidos = 0
        self.firma = None
        self.t_cambio = time.time()

    def leer(self):
        try:
            with SilenciarSalida():
                accel = tuple(self.bno.acceleration)
                q = tuple(self.bno.quaternion)  # (i, j, k, real)
            # La librería a veces deja pasar paquetes corruptos sin excepción
            # (por ejemplo Pitch = 90.0 exacto): un cuaternión válido tiene |q| = 1.
            norma = sum(c * c for c in q) ** 0.5
            if not abs(norma - 1.0) <= IMU_NORMA_TOL:
                raise ValueError(f"cuaternión inválido (|q|={norma:.3f})")
            euler = quat_to_euler(*q)
        except Exception as e:
            self.errores_seguidos += 1
            print(f"\n[IMU ERROR] {type(e).__name__}: {e}", file=sys.stderr)
            # OSError = el sensor dejó de contestar en el bus: reinicio inmediato.
            # Otros errores (paquete corrupto) suelen recuperarse solos; si se repiten, reinicio.
            if isinstance(e, OSError):
                self.reiniciar("OSError")
            elif self.errores_seguidos >= IMU_ERRORES_MAX:
                self.reiniciar(f"{self.errores_seguidos} errores seguidos")
            return self._muestra_invalida()

        self.errores_seguidos = 0
        self.ultimo_accel, self.ultimo_euler = accel, euler

        # IMU congelado: el sensor dejó de mandar reportes y la librería sigue
        # devolviendo el último valor guardado. Con ruido real del acelerómetro,
        # que accel + cuaternión sean idénticos durante segundos no es normal.
        firma = (accel, q)
        ahora = time.time()
        if firma != self.firma:
            self.firma = firma
            self.t_cambio = ahora
        elif ahora - self.t_cambio > IMU_CONGELADO_S:
            self.reiniciar("sin datos nuevos (IMU congelado)")
            return self._muestra_invalida()

        return dict(accel=accel, euler=euler, ok=True)


def init_radar(retries=4, contexto_guardado=None):
    print(f"Iniciando Radar XM125 por UART ({RADAR_PUERTO_SERIE})...")
    # Para limpiar el buffer antes de abrir el cliente se necesita un baudrate concreto
    baud_limpieza = RADAR_BAUDRATE or 115200

    for intento in range(1, retries + 1):
        s = None
        client = None
        try:
            # Limpieza previa del buffer UART
            s = serial.Serial(RADAR_PUERTO_SERIE, baudrate=baud_limpieza, timeout=0.5)
            s.reset_input_buffer()
            s.reset_output_buffer()
            s.read_all()
            s.close()
            time.sleep(0.5)

            client = a121.Client.open(
                serial_port=RADAR_PUERTO_SERIE,
                override_baudrate=RADAR_BAUDRATE,
                flow_control=RADAR_FLOW_CONTROL,
            )

            config = DetectorConfig(**RADAR_CONFIG)
            config.validate()

            detector = Detector(client=client, sensor_ids=[1], detector_config=config, context=contexto_guardado)
            time.sleep(0.3)

            if config.close_range_leakage_cancellation:
                if contexto_guardado is None:
                    print("  Calibrando close range inicial...")
                    detector.calibrate_detector()
                    # Guardamos el contexto para reutilizarlo en las reconexiones
                    contexto_guardado = detector.context
                else:
                    print("  [✓] Reutilizando calibración guardada.")

            detector.start()
            print(f"✓ OK: Radar listo [{config.start_m*100:.1f} cm - {config.end_m*100:.0f} cm] (intento {intento})")
            return client, detector, contexto_guardado

        except Exception as e:
            print(f"  Intento {intento}/{retries} falló: {type(e).__name__}: {e}")
            try:
                if s and s.is_open:
                    s.close()
            except Exception:
                pass
            try:
                if client:
                    client.close()
            except Exception:
                pass
            time.sleep(2.0)

    raise RuntimeError("No se pudo inicializar el radar tras varios intentos")


def leer_radar(detector):
    try:
        resultado = detector.get_next()
        distancias = resultado[1].distances
        if distancias is not None and len(distancias) > 0:
            return dict(distancia_m=float(distancias[0]), ok=True, desconectado=False)
        return dict(distancia_m=None, ok=True, desconectado=False)
    except Exception as e:
        if "No messages to consume" in str(e):
            return dict(distancia_m=None, ok=True, desconectado=False)

        # Cualquier otro error (como UnicodeDecodeError) rompe la sincronización:
        # marcamos 'desconectado=True' para obligar a que se reinicie el sensor.
        print(f"\n[RADAR ERROR] {type(e).__name__}: {e}", file=sys.stderr)
        return dict(distancia_m=None, ok=False, desconectado=True)


def abrir_csv(sid=None):
    nombre = f"datos_{sid or datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    f = open(nombre, "w", newline="")
    writer = csv.writer(f)
    writer.writerow([
        "timestamp", "accel_x", "accel_y", "accel_z",
        "roll", "pitch", "yaw", "distancia_m", "imu_ok", "radar_ok",
    ])
    print(f"Guardando respaldo local en: {nombre}")
    return f, writer


# -----------------------------------------------------------
def ip_local():
    """IP que usaría la Pi para salir a la red (no envía nada, solo consulta la ruta)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except Exception:
        return "desconocida"
    finally:
        s.close()


# ===========================================================
#  MODO MASTER / ESCLAVA: sesiones ON / OFF por el puerto 5004
# ===========================================================
# Estados:  IDLE -> PREPARANDO -> MIDIENDO -> CERRANDO -> IDLE
#   ON  : la Pico reinicia su IMU y avisa "listo"; la master inicia IMU + radar (con su
#         calibración de siempre); cuando las dos están listas la master fija T0 (en SU
#         reloj), se lo traduce al reloj de la Pico y las dos miden a partir de ahí.
#   OFF : las dos paran, la master une los datos y devuelve el CSV por el mismo puerto.
# La IMU y el radar los toca solo el hilo principal (como antes). El puerto de control
# corre en hilos aparte y le pasa ON/OFF por una cola.

SALIR = threading.Event()


class SesionError(Exception):
    pass


class SesionAbortada(Exception):
    def __init__(self, resp_off):
        super().__init__("abortada por OFF")
        self.resp_off = resp_off


class Live:
    """Transmisión en vivo a la PC (puerto 5005), igual que antes."""
    srv = None
    conn = None
    conn_file = None


class Control(threading.Thread):
    """Puerto 5004. Una línea de texto por conexión: ON, OFF, STATUS o CSV.

    Respuestas (una línea cada una, UTF-8):
      OK <texto>                    informativa (la sigue otra línea)
      LISTO t0=<epoch> sid=<sid>    ON: ya están midiendo
      FIN <texto>                   OFF sin CSV (sesión abortada)
      CSV <bytes> <nombre>\\n<datos> OFF / CSV: el archivo unificado
      ESTADO ...                    STATUS
      ERROR <texto>
    """

    def __init__(self, receptor, puerto=PUERTO_CONTROL):
        super().__init__(name="Control", daemon=True)
        self.receptor = receptor
        self.cola = queue.Queue()
        self.lock = threading.Lock()
        self.estado = "IDLE"
        self.sid = ""
        self.detalle = ""
        self.muestras = 0
        self.ultimo_csv = None
        self.contexto_radar = None
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("0.0.0.0", puerto))
        self.srv.listen(5)
        self.srv.settimeout(0.5)

    def set_estado(self, estado, sid=None, detalle=""):
        with self.lock:
            self.estado = estado
            if sid is not None:
                self.sid = sid
            self.detalle = detalle

    def run(self):
        while not SALIR.is_set():
            try:
                c, _ = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._atender, args=(c,), daemon=True).start()
        self.srv.close()

    # ---------------------------------------------------------------
    @staticmethod
    def _linea(c, texto):
        c.sendall((texto + "\n").encode())

    @staticmethod
    def _leer_comando(c):
        datos = b""
        try:
            while b"\n" not in datos and len(datos) < 64:
                trozo = c.recv(64)
                if not trozo:
                    break
                datos += trozo
        except socket.timeout:
            pass            # cliente que manda "ON" sin salto de línea: usar lo recibido
        return datos.decode(errors="replace").strip().upper()

    def _enviar_csv(self, c, ruta):
        with open(ruta, "rb") as f:
            datos = f.read()
        self._linea(c, f"CSV {len(datos)} {os.path.basename(ruta)}")
        c.sendall(datos)

    def _status(self):
        with self.lock:
            estado, sid, detalle = self.estado, self.sid, self.detalle
        r = self.receptor
        if r is None:
            pico = "sin_pico"
        else:
            pico = "conectada" if r.conectada else "DESCONECTADA"
            pico += f" pico_muestras={r.muestras_sesion} pico_perdidas={r.perdidas}"
        texto = (f"ESTADO {estado} sid={sid or '-'} pico={pico} "
                 f"muestras_master={self.muestras} ultimo_csv={self.ultimo_csv or '-'}")
        return texto + (f" detalle={detalle}" if detalle else "")

    def _atender(self, c):
        try:
            c.settimeout(10.0)
            cmd = self._leer_comando(c)
            if cmd == "STATUS":
                self._linea(c, self._status())
            elif cmd == "CSV":
                if self.ultimo_csv and os.path.exists(self.ultimo_csv):
                    self._enviar_csv(c, self.ultimo_csv)
                else:
                    self._linea(c, "ERROR todavía no hay ningún CSV")
            elif cmd in ("ON", "OFF"):
                with self.lock:
                    estado = self.estado
                if cmd == "ON" and estado != "IDLE":
                    self._linea(c, f"ERROR no se puede: estado actual {estado}")
                elif cmd == "OFF" and estado not in ("PREPARANDO", "MIDIENDO"):
                    self._linea(c, f"ERROR no hay medición activa (estado {estado})")
                else:
                    self._esperar_respuestas(c, cmd)
            else:
                self._linea(c, "ERROR comando desconocido (ON, OFF, STATUS, CSV)")
        except OSError:
            pass
        finally:
            try:
                c.close()
            except OSError:
                pass

    def _esperar_respuestas(self, c, cmd):
        resp = queue.Queue()
        self.cola.put((cmd, resp))
        c.settimeout(None)
        while True:
            try:
                msg = resp.get(timeout=1.0)
            except queue.Empty:
                if SALIR.is_set():
                    return
                continue
            if isinstance(msg, tuple):          # ("CSV", ruta)
                self._enviar_csv(c, msg[1])
                return
            self._linea(c, msg)
            if msg.split(" ", 1)[0] in ("LISTO", "ERROR", "FIN"):
                return


# -----------------------------------------------------------
def revisar_cola(ctl):
    """Si hay un OFF pendiente devuelve su cola de respuesta; cualquier ON se rechaza."""
    while True:
        try:
            cmd, resp = ctl.cola.get_nowait()
        except queue.Empty:
            return None
        if cmd == "OFF":
            return resp
        resp.put("ERROR ya hay una sesión en curso")


def esperar_pico_listo(ctl, receptor, sid):
    """Manda 'prep' (se repite cada 3 s) y espera el 'listo' de la Pico."""
    limite = time.time() + PICO_LISTO_TIMEOUT_S
    t_envio = 0.0
    while not receptor.ev_listo.is_set():
        resp_off = revisar_cola(ctl)
        if resp_off is not None:
            raise SesionAbortada(resp_off)
        if SALIR.is_set():
            raise SesionError("interrumpido")
        ahora = time.time()
        if ahora > limite:
            raise SesionError(f"la Pico no contestó 'listo' en {PICO_LISTO_TIMEOUT_S:.0f} s")
        if ahora - t_envio >= 3.0:
            receptor.enviar(f"prep {sid}")
            t_envio = ahora
        receptor.ev_listo.wait(0.1)
    if not receptor.listo.get("imu_ok"):
        raise SesionError("la IMU de la Pico no responde (cableado, dirección I2C 0x4A/0x4B)")


def esperar_pico_fin(receptor):
    """Manda 'stop' (se repite cada 2 s) hasta que la Pico confirme con 'fin'."""
    limite = time.time() + PICO_FIN_TIMEOUT_S
    t_envio = 0.0
    while not receptor.ev_fin.is_set() and time.time() < limite:
        if time.time() - t_envio >= 2.0:
            receptor.enviar("stop")
            t_envio = time.time()
        receptor.ev_fin.wait(0.1)
    return receptor.ev_fin.is_set()


def liberar(detector, client, imu_dev, csv_file):
    """Cierra radar, IMU y CSV (se puede llamar más de una vez)."""
    if csv_file is not None:
        try:
            csv_file.flush()
            csv_file.close()
        except Exception:
            pass
    for obj, metodo in ((detector, "stop"), (client, "close")):
        if obj is not None:
            try:
                getattr(obj, metodo)()
            except Exception:
                pass
    if imu_dev is not None:
        try:
            imu_dev.i2c.deinit()
        except Exception:
            pass


COLS_PICO = ["accel_x", "accel_y", "accel_z", "roll", "pitch", "yaw", "imu_ok"]


def unir_csv(sid, t0):
    """datos_<sid>.csv (master) + pico_<sid>.csv (Pico, ya en reloj de la master)
    -> medicion_<sid>.csv. Una fila por muestra de la master; a cada una se le pega la muestra
    de la Pico más cercana en el tiempo si está a menos de MERGE_TOL_S (si no, queda vacío).
    pico_dt_ms = tiempo de la Pico menos tiempo de la master (positivo: la Pico es posterior)."""
    ruta_m, ruta_p, ruta_out = f"datos_{sid}.csv", f"pico_{sid}.csv", f"medicion_{sid}.csv"
    with open(ruta_m, newline="") as f:
        lector = csv.DictReader(f)
        cols_m = lector.fieldnames
        filas_m = list(lector)
    pico = []
    if os.path.exists(ruta_p):
        with open(ruta_p, newline="") as f:
            for r in csv.DictReader(f):
                try:
                    pico.append((float(r["ts_master_est"]), r))
                except (KeyError, ValueError):
                    pass
        pico.sort(key=lambda x: x[0])
    tiempos = [p[0] for p in pico]
    con_pico = 0
    with open(ruta_out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_rel_s"] + cols_m + ["pico_ts", "pico_dt_ms"] + [f"pico_{c}" for c in COLS_PICO])
        for r in filas_m:
            ts = float(r["timestamp"])
            extra = [""] * (2 + len(COLS_PICO))
            if tiempos:
                i = bisect.bisect_left(tiempos, ts)
                j = min((k for k in (i - 1, i) if 0 <= k < len(tiempos)),
                        key=lambda k: abs(tiempos[k] - ts))
                dt = tiempos[j] - ts
                if abs(dt) <= MERGE_TOL_S:
                    p = pico[j][1]
                    extra = [f"{tiempos[j]:.4f}", f"{dt * 1000:.1f}"] + [p[c] for c in COLS_PICO]
                    con_pico += 1
            w.writerow([f"{ts - t0:.4f}"] + [r[c] for c in cols_m] + extra)
    return ruta_out, len(filas_m), con_pico


# -----------------------------------------------------------
def sesion(ctl, receptor, live, resp_on):
    """Una medición completa: de ON a OFF. Devuelve cuando la master vuelve a IDLE."""
    sid = datetime.now().strftime("%Y%m%d_%H%M%S")
    ctl.set_estado("PREPARANDO", sid)
    ctl.muestras = 0
    imu_dev = client = detector = csv_file = None
    contador = 0
    reconexiones_radar = 0
    resp_off = None
    t0 = None
    error = None
    try:
        # 1) Pico: fallar rápido si no está (antes de gastar ~30 s calibrando el radar)
        if receptor is not None:
            if not receptor.conectada:
                raise SesionError("la Pico no está conectada a la master "
                                  "(LED, WiFi, MASTER_IP en su config.py; o USAR_PICO=False)")
            receptor.iniciar_sesion(sid)
            esperar_pico_listo(ctl, receptor, sid)
        resp_on.put("OK Pico lista; iniciando IMU y calibrando radar "
                    "(con CLOSE_RANGE puede tardar ~30 s)...")

        # 2) Master: exactamente lo de siempre (IMU, pausa, radar con su calibración, CSV)
        imu_dev = Imu(crear_reset_imu())
        time.sleep(1.0)
        contexto = None if RECALIBRAR_CADA_ON else ctl.contexto_radar
        client, detector, contexto_radar = init_radar(contexto_guardado=contexto)
        ctl.contexto_radar = contexto_radar
        csv_file, csv_writer = abrir_csv(sid)

        resp_off = revisar_cola(ctl)        # ¿pidieron OFF mientras se calibraba?
        if resp_off is not None:
            raise SesionAbortada(resp_off)

        # 3) T0: instante común, en el reloj de la master, YA con la calibración terminada
        if receptor is not None:
            if not receptor.conectada:
                raise SesionError("la Pico se desconectó durante la calibración")
            off = receptor.offset()
            if off is None:
                raise SesionError("sin referencia de reloj de la Pico")
            t0 = time.time() + MARGEN_START_S
            orden_start = f"start {int(round((t0 - off) * 1000))}"
            if not receptor.enviar(orden_start):
                raise SesionError("no pude mandarle 'start' a la Pico")
        else:
            t0 = time.time()
        ctl.set_estado("MIDIENDO", sid)
        resp_on.put(f"LISTO t0={t0:.4f} sid={sid}")
        print(f"\n[SESIÓN {sid}] midiendo desde t0={t0:.3f}")

        # 4) Loop de medición: el de siempre. Hasta T0 se lee todo pero no se guarda
        #    (así el radar y la IMU no quedan sin leer durante el margen de arranque).
        conn, conn_file, srv = live.conn, live.conn_file, live.srv
        avisado = False
        try:
            while not SALIR.is_set():
                if not ctl.cola.empty():
                    resp_off = revisar_cola(ctl)
                    if resp_off is not None:
                        break

                # Aceptar conexión si todavía no hay una (retorna al instante si no hay nadie)
                if conn is None:
                    try:
                        conn, addr = srv.accept()
                        conn.setblocking(True)
                        conn_file = conn.makefile("w")
                        print(f"✓ PC conectada desde {addr}")
                    except BlockingIOError:
                        pass

                ts = time.time()
                grabar = ts >= t0

                t_a = time.perf_counter()
                imu = imu_dev.leer()
                t_b = time.perf_counter()
                radar = leer_radar(detector)
                t_c = time.perf_counter()

                # PROTECCIÓN ANTI-IMU: si la IMU tardó más de 1.5 s (porque se reinició),
                # es casi seguro que el buffer UART del radar se llenó de basura y perdió sincronía.
                # Forzamos una reconexión preventiva del radar.
                if (t_b - t_a) > 1.5:
                    print("\n[SISTEMA] IMU bloqueó el bucle por >1.5s. Limpiando y reiniciando radar...", file=sys.stderr)
                    radar["desconectado"] = True

                if radar["desconectado"]:
                    print("\n[RADAR] Reconectando...", file=sys.stderr)
                    reconexiones_radar += 1
                    try:
                        detector.stop()
                    except Exception:
                        pass
                    try:
                        client.close()
                    except Exception:
                        pass
                    try:
                        client, detector, contexto_radar = init_radar(contexto_guardado=contexto_radar)
                        print("[RADAR] Reconectado con éxito", file=sys.stderr)
                    except Exception as e:
                        print(f"[RADAR] Falló la reconexión: {e}", file=sys.stderr)
                        time.sleep(2.0)

                ax, ay, az = imu["accel"]
                roll, pitch, yaw = imu["euler"]
                dist_m = radar["distancia_m"]

                # Un solo diccionario, usado tanto para el JSON por TCP como para la fila del CSV.
                muestra = dict(
                    timestamp=ts,
                    accel_x=ax, accel_y=ay, accel_z=az,
                    roll=roll, pitch=pitch, yaw=yaw,
                    distancia_m=dist_m,
                    imu_ok=imu["ok"], radar_ok=radar["ok"],
                )

                t_d = t_e = t_c
                if grabar:
                    # Mandar a la PC si hay conexión activa
                    if conn is not None:
                        try:
                            conn_file.write(json.dumps(muestra) + "\n")
                            conn_file.flush()
                        except (BrokenPipeError, ConnectionResetError, OSError):
                            print("\n[!] PC desconectada, sigo midiendo y esperando reconexión...")
                            try:
                                conn.close()
                            except Exception:
                                pass
                            conn = None
                            conn_file = None

                    t_d = time.perf_counter()

                    # Respaldo local siempre, haya o no PC conectada
                    csv_writer.writerow([
                        f"{muestra['timestamp']:.4f}",
                        f"{muestra['accel_x']:.4f}", f"{muestra['accel_y']:.4f}", f"{muestra['accel_z']:.4f}",
                        f"{muestra['roll']:.3f}", f"{muestra['pitch']:.3f}", f"{muestra['yaw']:.3f}",
                        f"{muestra['distancia_m']:.4f}" if muestra['distancia_m'] is not None else "",
                        int(muestra['imu_ok']), int(muestra['radar_ok']),
                    ])

                    contador += 1
                    ctl.muestras = contador
                    if contador % 20 == 0:
                        csv_file.flush()

                    t_e = time.perf_counter()

                    if contador % 5 == 0:
                        print(
                            f"\n[TIEMPOS] IMU:{(t_b-t_a)*1000:6.1f}ms  "
                            f"Radar:{(t_c-t_b)*1000:6.1f}ms  "
                            f"TCP:{(t_d-t_c)*1000:6.1f}ms  "
                            f"CSV:{(t_e-t_d)*1000:6.1f}ms  "
                            f"Total:{(t_e-t_a)*1000:6.1f}ms",
                            file=sys.stderr,
                        )

                # Si la Pico no empezó a mandar muestras 2 s después de T0, se reintenta 'start'
                if receptor is not None and not avisado and ts > t0 + 2.0:
                    avisado = True
                    if receptor.muestras_sesion == 0:
                        print("\n[PICO] sin muestras 2 s después de T0: reenvío 'start'", file=sys.stderr)
                        receptor.enviar(orden_start)

                sys.stdout.write(f"\rMuestras: {contador}  |  Dist: "
                                 f"{f'{dist_m*100:.1f}cm' if dist_m else '------'}  |  "
                                 f"Roll:{roll:6.1f} Pitch:{pitch:6.1f} Yaw:{yaw:6.1f}  |  "
                                 f"IMU:{'ok' if imu['ok'] else 'X '}"
                                 + (f"  |  Pico:{receptor.estado()}" if receptor else ""))
                sys.stdout.flush()
        except Exception as e:
            error = e
            print(f"\n[SESIÓN] error en el loop de medición: {type(e).__name__}: {e}", file=sys.stderr)
        live.conn, live.conn_file = conn, conn_file

        # 5) Cierre: parar las dos, esperar los datos pendientes de la Pico y unir
        t_off = time.time()
        ctl.set_estado("CERRANDO", sid)
        reinicios_imu = imu_dev.reinicios
        liberar(detector, client, imu_dev, csv_file)
        detector = client = imu_dev = csv_file = None
        print(f"\n\nFinalizado. Muestras: {contador}")
        print(f"Reinicios de IMU: {reinicios_imu}  |  Reconexiones de radar: {reconexiones_radar}")

        fin_ok = True
        info_pico = None
        if receptor is not None:
            receptor.enviar("stop")
            fin_ok = esperar_pico_fin(receptor)
            info_pico = receptor.cerrar_sesion()
            print(receptor.resumen())
        ruta, n_filas, con_pico = unir_csv(sid, t0)
        ctl.ultimo_csv = ruta
        resumen = (f"sid={sid} duracion={t_off - t0:.1f}s filas_master={n_filas}")
        if receptor is not None:
            resumen += (f" filas_con_pico={con_pico} pico_muestras={info_pico['muestras']}"
                        f" pico_perdidas={info_pico['perdidas']}")
            if not fin_ok:
                resumen += " AVISO=la_Pico_no_confirmo_el_fin"
        if error is not None:
            resumen += f" AVISO=error_en_loop:{type(error).__name__}"
        print(f"[SESIÓN {sid}] {resumen}\n  -> {ruta}")
        if resp_off is not None:
            resp_off.put(f"OK {resumen}")
            resp_off.put(("CSV", ruta))

    except SesionAbortada as e:
        resp_on.put("ERROR abortado por OFF antes de empezar a medir")
        e.resp_off.put("FIN sesión abortada antes de medir; no hay CSV")
        _abortar(receptor, detector, client, imu_dev, csv_file)
    except Exception as e:
        msg = f"{type(e).__name__}: {e}"
        print(f"\n[SESIÓN {sid}] ERROR {msg}", file=sys.stderr)
        resp_on.put(f"ERROR {msg}")
        if resp_off is not None:
            resp_off.put(f"ERROR {msg}")
        ctl.detalle = f"error:{msg}"
        _abortar(receptor, detector, client, imu_dev, csv_file)
    finally:
        # lo último: dejar a la master lista para otro ON
        detalle = ctl.detalle
        ctl.set_estado("IDLE", detalle=detalle if detalle.startswith("error:") else "")


def _abortar(receptor, detector, client, imu_dev, csv_file):
    liberar(detector, client, imu_dev, csv_file)
    if receptor is not None:
        receptor.enviar("stop")
        receptor.cerrar_sesion()


# -----------------------------------------------------------
def main():
    receptor = None
    if USAR_PICO:
        receptor = ReceptorPico()
        receptor.start()
    ctl = Control(receptor)
    ctl.start()

    # Transmisión en vivo a la PC (puerto 5005): se crea una vez, como antes
    live = Live()
    live.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    live.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    live.srv.bind(("0.0.0.0", PUERTO))
    live.srv.listen(1)
    live.srv.setblocking(False)  # accept() no bloqueante: nunca frena el loop de muestreo
    host = socket.gethostname()
    print(f"En vivo para la PC (opcional): {host}.local:{PUERTO}  |  {ip_local()}:{PUERTO}")
    print(f"Control: {ip_local()}:{PUERTO_CONTROL}  (ON / OFF / STATUS / CSV)")

    def salir(sig, frame):
        print("\nDeteniendo...")
        SALIR.set()

    signal.signal(signal.SIGINT, salir)
    signal.signal(signal.SIGTERM, salir)

    print("Master lista, esperando ON...")
    while not SALIR.is_set():
        try:
            cmd, resp = ctl.cola.get(timeout=0.2)
        except queue.Empty:
            continue
        if cmd == "ON":
            sesion(ctl, receptor, live, resp)
        else:
            resp.put("FIN no hay medición en curso")

    try:
        if live.conn:
            live.conn.close()
        live.srv.close()
    except Exception:
        pass
    if receptor is not None:
        receptor.detener()


if __name__ == "__main__":
    main()