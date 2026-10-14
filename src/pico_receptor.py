#!/usr/bin/env python3
"""
Corre en la Raspberry Pi Zero (master), en un hilo aparte dentro de pi_master.py.
También se puede correr solo (python3 pico_receptor.py) para probar la Pico sin radar.

Escucha a la Pico W en el puerto PICO_PORT (5006). La Pico manda una línea JSON por muestra:
  - al conectarse:  {"tipo": "hola", "boot_id": "...", "imu": "sim", "hz": 20, "t_pico_ms": t}
  - cada muestra:   {"seq": n, "t_pico_ms": t, "accel_x": ..., "roll": ..., "imu_ok": 1}
y la Zero le contesta "ack <seq>\n" cada ACK_S con el último seq guardado. La Pico conserva
las muestras hasta que se las confirman: si la conexión se corta, lo que estaba en vuelo
(que TCP ya había aceptado pero no llegó) se reenvía, y acá se descartan los repetidos.

Todo se guarda en un CSV propio (pico_YYYYmmdd_HHMMSS.csv) junto al CSV del master, así
se extraen juntos por SSH.

Sincronización de relojes: la Pico no tiene hora real, solo t_pico_ms (ms desde que arrancó).
Para llevar cada muestra al reloj del master se usa
    offset = t_recepcion - t_pico      (latencia de red + diferencia de relojes)
y el mínimo de offset en una ventana deslizante: la muestra que llegó más rápido es la que
mejor representa la diferencia de relojes (las demás están infladas por demoras de WiFi
o por haber quedado en el buffer de la Pico). ts_master_est = t_pico + min(offset).
"""

import csv
import json
import select
import socket
import sys
import threading
import time
from collections import deque
from datetime import datetime

PICO_HOST = "0.0.0.0"
PICO_PORT = 5006
VENTANA_SYNC = 400          # (~20 s a 20 Hz)
TIMEOUT_SIN_DATOS_S = 5.0   
                            
ACK_S = 0.25                

COLUMNAS = [
    "ts_master_est", "t_recepcion", "t_pico_ms", "seq", "boot_id",
    "accel_x", "accel_y", "accel_z", "roll", "pitch", "yaw", "imu_ok", "imu_tipo",
]


class ReceptorPico(threading.Thread):
    def __init__(self, host=PICO_HOST, port=PICO_PORT, prefijo_csv="pico"):
        super().__init__(name="ReceptorPico", daemon=True)
        self.host = host
        self.port = port
        self.prefijo_csv = prefijo_csv
        self._detener = threading.Event()

        self.conectada = False
        self.muestras = 0
        self.perdidas = 0       
        self.duplicadas = 0     # (se descartan)
        self.invalidas = 0      
        self.conexiones = 0
        self.ultima = None      

        self.boot_id = None
        self.imu_tipo = ""
        self.ultimo_seq = -1
        self.offsets = deque(maxlen=VENTANA_SYNC)

   
    def detener(self):
        self._detener.set()
        self.join(timeout=3.0)

    def estado(self):
        """Texto corto para la línea de estado del master."""
        if not self.conectada:
            return "--"
        return f"{self.muestras}"

    def resumen(self):
        return (f"Pico: muestras {self.muestras} | perdidas {self.perdidas} | "
                f"duplicadas {self.duplicadas} | líneas inválidas {self.invalidas} | "
                f"conexiones {self.conexiones}")

   
    def run(self):
        nombre = f"{self.prefijo_csv}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        f = open(nombre, "w", newline="")
        writer = csv.writer(f)
        writer.writerow(COLUMNAS)
        print(f"Guardando datos de la Pico en: {nombre}")

        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(1)
        print(f"Esperando a la Pico en el puerto {self.port}...")

        conn = None
        resto = b""
        t_ultimo_dato = 0.0
        t_ack = 0.0
        ack_enviado = -1
        sin_flush = 0

        try:
            while not self._detener.is_set():
                entradas = [srv] if conn is None else [srv, conn]
                listos, _, _ = select.select(entradas, [], [], 0.5)

                # Conexión nueva. Si ya había una, gana la nueva
                if srv in listos:
                    nueva, addr = srv.accept()
                    if conn is not None:
                        print("\n[PICO] Conexión nueva, descarto la anterior", file=sys.stderr)
                        conn.close()
                    conn = nueva
                    resto = b""
                    ack_enviado = -1
                    t_ultimo_dato = time.time()
                    self.conectada = True
                    self.conexiones += 1
                    print(f"\n✓ Pico conectada desde {addr}", file=sys.stderr)
                    continue

                if conn is None:
                    continue

                if conn in listos:
                    try:
                        datos = conn.recv(4096)
                    except OSError:
                        datos = b""
                    if not datos:
                        self._cerrar(conn, "la Pico cerró la conexión")
                        conn = None
                        continue
                    t_rx = time.time()
                    t_ultimo_dato = t_rx
                    *lineas, resto = (resto + datos).split(b"\n")
                    for linea in lineas:
                        fila = self._procesar(linea, t_rx)
                        if fila is not None:
                            writer.writerow(fila)
                            sin_flush += 1
                    if sin_flush >= 20:
                        f.flush()
                        sin_flush = 0
                    if self.ultimo_seq != ack_enviado and t_rx - t_ack >= ACK_S:
                        try:
                            conn.sendall(f"ack {self.ultimo_seq}\n".encode())
                        except OSError:
                            self._cerrar(conn, "error al mandar ack")
                            conn = None
                            continue
                        t_ack = t_rx
                        ack_enviado = self.ultimo_seq
                elif time.time() - t_ultimo_dato > TIMEOUT_SIN_DATOS_S:
                    self._cerrar(conn, f"sin datos hace {TIMEOUT_SIN_DATOS_S:.0f} s")
                    conn = None
        finally:
            if conn is not None:
                conn.close()
            srv.close()
            f.flush()
            f.close()

    def _cerrar(self, conn, motivo):
        print(f"\n[PICO] Desconectada ({motivo}), espero reconexión...", file=sys.stderr)
        try:
            conn.close()
        except Exception:
            pass
        self.conectada = False

    def _procesar(self, linea, t_rx):
        """Devuelve la fila del CSV, o None si la línea no es una muestra nueva."""
        try:
            d = json.loads(linea)
        except ValueError:
            self.invalidas += 1
            return None

        if d.get("tipo") == "hola":
            if d.get("boot_id") != self.boot_id:
                # La Pico arrancó de nuevo: su reloj y su seq empiezan de cero.
                self.boot_id = d.get("boot_id")
                self.ultimo_seq = -1
                self.offsets.clear()
            self.imu_tipo = d.get("imu", "")
            if "t_pico_ms" in d:
                self.offsets.append(t_rx - d["t_pico_ms"] / 1000.0)
            print(f"\n[PICO] boot_id={self.boot_id} imu={self.imu_tipo} hz={d.get('hz')}",
                  file=sys.stderr)
            if self.imu_tipo == "sim":
                print("[PICO] ¡OJO! La Pico manda datos SIMULADOS, no de una IMU real",
                      file=sys.stderr)
            return None

        try:
            seq = int(d["seq"])
            t_pico = d["t_pico_ms"] / 1000.0
        except (KeyError, TypeError, ValueError):
            self.invalidas += 1
            return None

        if seq <= self.ultimo_seq:
            self.duplicadas += 1
            return None
        if self.ultimo_seq >= 0 and seq > self.ultimo_seq + 1:
            self.perdidas += seq - self.ultimo_seq - 1
        self.ultimo_seq = seq

        self.offsets.append(t_rx - t_pico)
        ts_est = t_pico + min(self.offsets)

        self.muestras += 1
        self.ultima = d
        return [
            f"{ts_est:.4f}", f"{t_rx:.4f}", d["t_pico_ms"], seq, self.boot_id,
            d.get("accel_x", ""), d.get("accel_y", ""), d.get("accel_z", ""),
            d.get("roll", ""), d.get("pitch", ""), d.get("yaw", ""),
            int(bool(d.get("imu_ok", 0))), self.imu_tipo,
        ]



if __name__ == "__main__":
    receptor = ReceptorPico()
    receptor.start()
    try:
        while receptor.is_alive():
            time.sleep(1.0)
            u = receptor.ultima or {}
            sys.stdout.write(
                f"\rPico: {receptor.estado():>6}  |  perdidas {receptor.perdidas}  |  "
                f"Roll:{u.get('roll', 0):6.1f} Pitch:{u.get('pitch', 0):6.1f} "
                f"Yaw:{u.get('yaw', 0):6.1f}   "
            )
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    receptor.detener()
    print("\n" + receptor.resumen())
