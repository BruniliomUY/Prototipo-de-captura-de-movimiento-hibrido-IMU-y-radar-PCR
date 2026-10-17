#!/usr/bin/env python3
"""
Control remoto de la master (Raspberry Pi Zero) desde la Mac. Habla con el puerto 5004.

    python3 controlar.py on     [--host IP]   arranca: calibra y empieza a medir (espera hasta que midan)
    python3 controlar.py off    [--host IP]   para, y baja medicion_<sid>.csv a esta carpeta
    python3 controlar.py status [--host IP]
    python3 controlar.py csv    [--host IP]   vuelve a bajar el último CSV

La IP de la Zero: --host, o la variable de entorno ZERO_HOST (se ve con `hostname -I` en la Zero).
Sin instalar nada también sirve:  printf 'ON\n' | nc IP 5004
"""

import argparse
import os
import socket
import sys

PUERTO = 5004


def pedir(host, comando, espera_s, carpeta):
    try:
        s = socket.create_connection((host, PUERTO), timeout=10)
    except OSError as e:
        sys.exit(f"No pude conectar a {host}:{PUERTO} ({e}).\n"
                 "¿Está corriendo master.py en la Zero? ¿La IP es correcta?")
    s.settimeout(espera_s)
    s.sendall((comando.upper() + "\n").encode())
    f = s.makefile("rb")
    try:
        while True:
            linea = f.readline()
            if not linea:
                print("(la master cerró la conexión)")
                return 1
            texto = linea.decode(errors="replace").rstrip("\n")
            print(texto)
            palabra = texto.split(" ", 1)[0]
            if palabra == "CSV":
                _, n, nombre = texto.split(" ", 2)
                datos = f.read(int(n))
                if len(datos) != int(n):
                    print(f"CSV incompleto: llegaron {len(datos)} de {n} bytes")
                    return 1
                os.makedirs(carpeta, exist_ok=True)
                ruta = os.path.join(carpeta, nombre)
                with open(ruta, "wb") as out:
                    out.write(datos)
                print(f"-> guardado en {os.path.abspath(ruta)} ({len(datos)} bytes)")
                return 0
            if palabra in ("LISTO", "ESTADO", "FIN"):
                return 0
            if palabra == "ERROR":
                return 1
    except socket.timeout:
        print(f"Sin respuesta en {espera_s:.0f} s (la master puede seguir trabajando: probá `status`).")
        return 1
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser(description="Control de la master (puerto 5004)")
    ap.add_argument("comando", choices=["on", "off", "status", "csv"])
    ap.add_argument("--host", default=os.environ.get("ZERO_HOST", "raspberrypi.local"))
    ap.add_argument("--carpeta", default=".", help="dónde guardar el CSV (por defecto, la carpeta actual)")
    a = ap.parse_args()
    espera = {"on": 300, "off": 120, "status": 15, "csv": 120}[a.comando]
    sys.exit(pedir(a.host, a.comando, espera, a.carpeta))


if __name__ == "__main__":
    main()