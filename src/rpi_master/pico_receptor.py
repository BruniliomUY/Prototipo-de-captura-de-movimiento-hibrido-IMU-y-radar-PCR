#!/usr/bin/env python3
"""
Corre en la Raspberry Pi Zero (master), en un hilo aparte dentro de master.py.
También se puede correr solo (python3 pico_receptor.py) para probar la Pico sin radar:
hace una sesión completa (prep -> start -> ... Ctrl+C -> stop) y deja pico_<fecha>.csv.

El hilo vive desde que arranca la master (la Pico se conecta y queda en reposo mandando
latidos). Los datos solo se guardan dentro de una SESIÓN: iniciar_sesion(sid) abre
pico_<sid>.csv y cerrar_sesion() lo cierra. La master le da órdenes a la Pico por el mismo
socket con enviar():  "prep <sid>" (reinicia la IMU y contesta "listo"), "start <t0_pico_ms>"
(empieza a medir en ese instante de su reloj) y "stop" (deja de medir, vacía lo pendiente y
contesta "fin").

Escucha a la Pico W en el puerto PICO_PORT (5006). La Pico manda una línea JSON por mensaje:
  - al conectarse:  {"tipo": "hola", "boot_id": "...", "imu": "sim", "hz": 20, "t_pico_ms": t}
  - cada 250 ms:    {"tipo": "hb", "t_pico_ms": t}            (latido; también sirve para el reloj)
  - al preparar:    {"tipo": "listo", "sid": ..., "imu_ok": 1, ...}
  - cada muestra:   {"seq": n, "t_pico_ms": t, "accel_x": ..., "roll": ..., "imu_ok": 1}
  - al terminar:    {"tipo": "fin", "sid": ..., "ultimo_seq": n, ...}
y la Zero le contesta "ack <seq>\n" cada ACK_S con el último seq guardado. La Pico conserva
las muestras hasta que se las confirman: si la conexión se corta, lo que estaba en vuelo
(que TCP ya había aceptado pero no llegó) se reenvía, y acá se descartan los repetidos.

Cada sesión se guarda en pico_<sid>.csv junto al datos_<sid>.csv del master.

Sincronización de relojes: la Pico no tiene hora real, solo t_pico_ms (ms desde que arrancó).
Para llevar cada muestra al reloj del master se usa
    offset = t_recepcion - t_pico      (latencia de red + diferencia de relojes)
y el mínimo de offset en una ventana deslizante: la muestra que llegó más rápido es la que
mejor representa la diferencia de relojes (las demás están infladas por demoras de WiFi
o por haber quedado en el buffer de la Pico). ts_master_est = t_pico + min(offset).
El "hola" trae el reloj de la Pico en el momento de conectar, así que las muestras que
quedaron en el buffer mientras no había WiFi ya se ubican bien.
La ventana deslizante absorbe la deriva del cristal de la Pico. Se guardan también
t_recepcion y t_pico_ms crudos para poder rehacer la sincronización al analizar.
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
VENTANA_SYNC = 400          # muestras para estimar el offset de reloj (~20 s a 20 Hz)
TIMEOUT_SIN_DATOS_S = 5.0   # sin datos este tiempo => se da la conexión por muerta
                            # (si la Pico se queda sin batería el TCP no se cierra solo)
ACK_S = 0.25                # cada cuánto confirmarle a la Pico lo recibido

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
        self.perdidas = 0       # huecos en seq (la Pico descartó por buffer lleno, o se perdieron)
        self.duplicadas = 0     # reenvíos tras reconexión (se descartan)
        self.invalidas = 0      # líneas que no son JSON (cortadas en una desconexión)
        self.conexiones = 0
        self.ultima = None      # última muestra recibida (dict)

        self.boot_id = None
        self.imu_tipo = ""
        self.ultimo_seq = -1
        self.offsets = deque(maxlen=VENTANA_SYNC)

        # --- sesión ---
        self.sid = None
        self.csv_nombre = None
        self.muestras_sesion = 0
        self.listo = None       # último mensaje "listo" de esta sesión
        self.fin = None         # último mensaje "fin" de esta sesión
        self.ev_listo = threading.Event()
        self.ev_fin = threading.Event()
        self._lock = threading.Lock()        # protege el CSV de la sesión
        self._send_lock = threading.Lock()   # un solo hilo a la vez escribiendo en el socket
        self._conn = None
        self._f = None
        self._w = None

    # ---------------------------------------------------------------
    def offset(self):
        """Mejor estimación (segundos) de: reloj_master = t_pico + offset. None si no hay datos."""
        for _ in range(3):
            try:
                return min(self.offsets)
            except ValueError:
                return None
            except RuntimeError:    # el hilo agregó un valor justo mientras se iteraba
                continue
        return None

    def enviar(self, texto):
        """Manda una orden (una línea) a la Pico. False si no hay conexión."""
        conn = self._conn
        if conn is None:
            return False
        try:
            with self._send_lock:
                conn.sendall((texto + "\n").encode())
            return True
        except OSError:
            return False

    def iniciar_sesion(self, sid):
        """Abre pico_<sid>.csv y deja los contadores en cero. Llamar antes de mandar 'prep'."""
        with self._lock:
            self._cerrar_csv()
            self.csv_nombre = f"{self.prefijo_csv}_{sid}.csv"
            self._f = open(self.csv_nombre, "w", newline="")
            self._w = csv.writer(self._f)
            self._w.writerow(COLUMNAS)
            self.sid = sid
            self.muestras_sesion = 0
            self.perdidas = self.duplicadas = self.invalidas = 0
            self.ultimo_seq = -1
            self.listo = None
            self.fin = None
            self.ev_listo.clear()
            self.ev_fin.clear()

    def cerrar_sesion(self):
        """Cierra el CSV de la sesión y devuelve un dict con los contadores."""
        with self._lock:
            self._cerrar_csv()
            return dict(muestras=self.muestras_sesion, perdidas=self.perdidas,
                        duplicadas=self.duplicadas, invalidas=self.invalidas,
                        fin=self.fin, csv=self.csv_nombre)

    def _cerrar_csv(self):
        if self._f is not None:
            try:
                self._f.flush()
                self._f.close()
            except Exception:
                pass
        self._f = None
        self._w = None

    # ---------------------------------------------------------------
    def detener(self):
        self._detener.set()
        self.join(timeout=3.0)
        with self._lock:
            self._cerrar_csv()

    def estado(self):
        """Texto corto para la línea de estado del master."""
        if not self.conectada:
            return "--"
        return f"{self.muestras_sesion}"

    def resumen(self):
        return (f"Pico: muestras {self.muestras_sesion} | perdidas {self.perdidas} | "
                f"duplicadas {self.duplicadas} | líneas inválidas {self.invalidas} | "
                f"conexiones {self.conexiones}")

    # ---------------------------------------------------------------
    def run(self):
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

                # Conexión nueva. Si ya había una, gana la nueva: lo normal es que la Pico
                # se haya reiniciado y la vieja haya quedado colgada.
                if srv in listos:
                    nueva, addr = srv.accept()
                    if conn is not None:
                        print("\n[PICO] Conexión nueva, descarto la anterior", file=sys.stderr)
                        conn.close()
                    conn = nueva
                    self._conn = conn
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
                        conn = self._conn = None
                        continue
                    t_rx = time.time()
                    t_ultimo_dato = t_rx
                    *lineas, resto = (resto + datos).split(b"\n")
                    for linea in lineas:
                        fila = self._procesar(linea, t_rx)
                        if fila is not None:
                            with self._lock:
                                if self._w is not None:     # solo se guarda dentro de una sesión
                                    self._w.writerow(fila)
                                    self.muestras_sesion += 1
                                    sin_flush += 1
                    if sin_flush >= 20:
                        with self._lock:
                            if self._f is not None:
                                self._f.flush()
                        sin_flush = 0
                    if self.ultimo_seq != ack_enviado and t_rx - t_ack >= ACK_S:
                        try:
                            with self._send_lock:
                                conn.sendall(f"ack {self.ultimo_seq}\n".encode())
                        except OSError:
                            self._cerrar(conn, "error al mandar ack")
                            conn = self._conn = None
                            continue
                        t_ack = t_rx
                        ack_enviado = self.ultimo_seq
                elif time.time() - t_ultimo_dato > TIMEOUT_SIN_DATOS_S:
                    self._cerrar(conn, f"sin datos hace {TIMEOUT_SIN_DATOS_S:.0f} s")
                    conn = self._conn = None
        finally:
            self._conn = None
            if conn is not None:
                conn.close()
            srv.close()

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

        tipo = d.get("tipo")
        if tipo in ("hb", "listo", "fin"):
            if tipo == "hb" or tipo == "listo":
                if "t_pico_ms" in d:
                    self.offsets.append(t_rx - d["t_pico_ms"] / 1000.0)
            if tipo == "listo" and d.get("sid") == self.sid:
                self.listo = d
                self.ev_listo.set()
            elif tipo == "fin" and d.get("sid") == self.sid:
                self.fin = d
                self.ev_fin.set()
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


# -----------------------------------------------------------
if __name__ == "__main__":
    # Prueba de la Pico sola (sin radar): una sesión completa a mano.
    receptor = ReceptorPico()
    receptor.start()
    print("Modo prueba: esperando a la Pico (Ctrl+C para terminar)...")
    try:
        while not receptor.conectada:
            time.sleep(0.5)
        sid = datetime.now().strftime("%Y%m%d_%H%M%S")
        receptor.iniciar_sesion(sid)
        receptor.enviar(f"prep {sid}")
        if not receptor.ev_listo.wait(15.0):
            print("La Pico no contestó 'listo' en 15 s")
        elif not receptor.listo.get("imu_ok"):
            print("La Pico contestó pero su IMU no responde:", receptor.listo)
        else:
            t0 = time.time() + 1.0
            receptor.enviar(f"start {int(round((t0 - receptor.offset()) * 1000))}")
            print(f"Midiendo desde t0={t0:.3f}. Ctrl+C para parar.")
            while receptor.is_alive():
                time.sleep(1.0)
                u = receptor.ultima or {}
                sys.stdout.write(
                    f"\rPico: {receptor.estado():>6}  |  perdidas {receptor.perdidas}  |  "
                    f"Roll:{u.get('roll', 0):6.1f} Pitch:{u.get('pitch', 0):6.1f} "
                    f"Yaw:{u.get('yaw', 0):6.1f}   ")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    receptor.enviar("stop")
    receptor.ev_fin.wait(10.0)
    info = receptor.cerrar_sesion()
    receptor.detener()
    print("\n" + receptor.resumen(), "| archivo:", info["csv"])