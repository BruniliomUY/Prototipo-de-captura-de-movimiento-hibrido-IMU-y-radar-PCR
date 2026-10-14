#!/usr/bin/env python3
"""
Corre en la Raspberry Pi.
Lee IMU (BNO085) + Radar (Acconeer XM125), guarda CSV local como respaldo,
y transmite cada muestra por TCP (JSON, una línea por muestra) a quien se conecte.

I2C del BNO085: usa clock stretching y la Pi lo maneja mal (KeyError / OSError al
leer). La velocidad real del bus se fija en /boot/firmware/config.txt (antes de
Bookworm: /boot/config.txt), no desde Python (Blinka ignora el parámetro frequency):
    dtparam=i2c_arm_baudrate=400000
y hay que reiniciar la Pi para que tome efecto.
"""

import time
import math
import csv
import json
import socket
import signal
import sys
import logging
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

from pico_receptor import ReceptorPico

from acconeer.exptool import a121
from acconeer.exptool.a121.algo.distance import (
    Detector,
    DetectorConfig,
    PeakSortingMethod,
    ReflectorShape,
    ThresholdMethod,
)

HOST = "0.0.0.0"   
PORT = 5005         
USAR_PICO = True    

# --- IMU: validación y recuperación automática ---
IMU_RESET_PIN = None    
IMU_NORMA_TOL = 0.02   
IMU_ERRORES_MAX = 3     
IMU_CONGELADO_S = 5.0   
                        

# --- Radar: configuración del detector de distancia ---

CLOSE_RANGE = True

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

RADAR_BAUDRATE = 115200       
RADAR_FLOW_CONTROL = True   



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
            time.sleep(0.5)  
        except Exception as e:
            print(f"[IMU] Falló el reinicio: {e}", file=sys.stderr)
            time.sleep(2.0)  
        self.errores_seguidos = 0
        self.firma = None
        self.t_cambio = time.time()

    def leer(self):
        try:
            with SilenciarSalida():
                accel = tuple(self.bno.acceleration)
                q = tuple(self.bno.quaternion)  # (i, j, k, real)
            norma = sum(c * c for c in q) ** 0.5
            if not abs(norma - 1.0) <= IMU_NORMA_TOL:
                raise ValueError(f"cuaternión inválido (|q|={norma:.3f})")
            euler = quat_to_euler(*q)
        except Exception as e:
            self.errores_seguidos += 1
            print(f"\n[IMU ERROR] {type(e).__name__}: {e}", file=sys.stderr)
            
            if isinstance(e, OSError):
                self.reiniciar("OSError")
            elif self.errores_seguidos >= IMU_ERRORES_MAX:
                self.reiniciar(f"{self.errores_seguidos} errores seguidos")
            return self._muestra_invalida()

        self.errores_seguidos = 0
        self.ultimo_accel, self.ultimo_euler = accel, euler


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
    print("Iniciando Radar XM125 por UART (/dev/serial0)...")
    for intento in range(1, retries + 1):
        s = None
        client = None
        try:
            s = serial.Serial("/dev/serial0", baudrate=115200, timeout=0.5)
            s.reset_input_buffer()
            s.reset_output_buffer()
            s.read_all()
            s.close()
            time.sleep(0.5)

            client = a121.Client.open(
                serial_port="/dev/serial0",
                override_baudrate=115200,
                flow_control=True,
            )

            config = DetectorConfig(**RADAR_CONFIG)
            config.validate()

            detector = Detector(client=client, sensor_ids=[1], detector_config=config, context=contexto_guardado)
            time.sleep(0.3)
            
            if config.close_range_leakage_cancellation:
                if contexto_guardado is None:
                    print("  Calibrando close range inicial (muy rápido)...")
                    detector.calibrate_detector()
                    # MAGIA 3: Guardamos el contexto para la próxima vez
                    contexto_guardado = detector.context
                else:
                    print("  [✓] Reutilizando calibración guardada. ¡Arranque instantáneo!")

            detector.start()
            print(f"✓ OK: Radar listo [{config.start_m*100:.1f} cm - {config.end_m*100:.0f} cm] (intento {intento})")
            return client, detector, contexto_guardado
            
        except Exception as e:
            print(f"  Intento {intento}/{retries} falló: {type(e).__name__}: {e}")
            try:
                if s and s.is_open: s.close()
            except: pass
            try:
                if client: client.close()
            except: pass
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
        

        print(f"\n[RADAR ERROR] {type(e).__name__}: {e}", file=sys.stderr)
        return dict(distancia_m=None, ok=False, desconectado=True)

def abrir_csv():
    nombre = f"datos_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    f = open(nombre, "w", newline="")
    writer = csv.writer(f)
    writer.writerow([
        "timestamp", "accel_x", "accel_y", "accel_z",
        "roll", "pitch", "yaw", "distancia_m", "imu_ok", "radar_ok",
    ])
    print(f"Guardando respaldo local en: {nombre}")
    return f, writer



def main():
    imu_dev = Imu(crear_reset_imu())
    time.sleep(1.0)
    contexto_radar = None
    client, detector, contexto_radar = init_radar(contexto_guardado=contexto_radar)
    csv_file, csv_writer = abrir_csv()

    receptor_pico = None
    if USAR_PICO:
        receptor_pico = ReceptorPico()
        receptor_pico.start()

    # Socket TCP: espera q PC se conecte
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print(f"\nEsperando conexión de la PC en el puerto {PORT}...")
    print("(mientras tanto ya estoy midiendo y grabando el CSV local)")

    conn = None
    conn_file = None

    running = True

    def salir(sig, frame):
        nonlocal running
        print("\nDeteniendo...")
        running = False

    signal.signal(signal.SIGINT, salir)
    signal.signal(signal.SIGTERM, salir)

    srv.setblocking(False)  

    contador = 0
    reconexiones_radar = 0

    while running:
        # Aceptar conexión si todavía no hay una 
        if conn is None:
            try:
                conn, addr = srv.accept()
                conn.setblocking(True)
                conn_file = conn.makefile("w")
                print(f"✓ PC conectada desde {addr}")
            except BlockingIOError:
                pass

        ts = time.time()

        t_a = time.perf_counter()
        imu = imu_dev.leer()
        t_b = time.perf_counter()
        radar = leer_radar(detector)
        t_c = time.perf_counter()


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

        muestra = dict(
            timestamp=ts,
            accel_x=ax, accel_y=ay, accel_z=az,
            roll=roll, pitch=pitch, yaw=yaw,
            distancia_m=dist_m,
            imu_ok=imu["ok"], radar_ok=radar["ok"],
        )

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

        # Respaldo local 
        csv_writer.writerow([
            f"{muestra['timestamp']:.4f}",
            f"{muestra['accel_x']:.4f}", f"{muestra['accel_y']:.4f}", f"{muestra['accel_z']:.4f}",
            f"{muestra['roll']:.3f}", f"{muestra['pitch']:.3f}", f"{muestra['yaw']:.3f}",
            f"{muestra['distancia_m']:.4f}" if muestra['distancia_m'] is not None else "",
            int(muestra['imu_ok']), int(muestra['radar_ok']),
        ])

        contador += 1
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

        sys.stdout.write(f"\rMuestras: {contador}  |  Dist: "
                          f"{f'{dist_m*100:.1f}cm' if dist_m else '------'}  |  "
                          f"Roll:{roll:6.1f} Pitch:{pitch:6.1f} Yaw:{yaw:6.1f}  |  "
                          f"IMU:{'ok' if imu['ok'] else 'X '}"
                          + (f"  |  Pico:{receptor_pico.estado()}" if receptor_pico else ""))
        sys.stdout.flush()

    print(f"\n\nFinalizado. Muestras: {contador}")
    print(f"Reinicios de IMU: {imu_dev.reinicios}  |  Reconexiones de radar: {reconexiones_radar}")
    if receptor_pico:
        receptor_pico.detener()
        print(receptor_pico.resumen())
    csv_file.flush()
    csv_file.close()
    try:
        if conn:
            conn.close()
        srv.close()
        detector.stop()
        client.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()
