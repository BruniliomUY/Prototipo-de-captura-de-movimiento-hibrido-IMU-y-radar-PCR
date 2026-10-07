"""
Corre en la Raspberry Pi Pico W (MicroPython).
Se conecta a la red WiFi de la Raspberry Pi principal (PI_radar) y le manda
su propio IMU por TCP, al puerto PICO_PORT (5006), como una línea JSON por muestra.

Ajustá SSID/PASSWORD y PI_IP a tu red y a la IP real de la Raspberry Pi.
Ajustá también la sección de lectura del IMU a la librería/sensor que uses
en esta segunda placa (puede no ser el mismo BNO085 de la Pi principal).
"""

import network
import socket
import time
import json
import machine

# --- Configuración de red ---
SSID = "PI_radar"
PASSWORD = "TuNuevaContraseñaSegura123"   # la misma que le pusiste al hotspot
PI_IP = "192.168.4.1"                     # IP de la Raspberry Pi en su propio hotspot
PI_PORT = 5006

# --- Configuración de muestreo ---
INTERVALO_S = 0.066  # ~15Hz, igual que el radar de la Pi principal


def conectar_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Conectando a WiFi:", SSID)
        wlan.connect(SSID, PASSWORD)
        t0 = time.time()
        while not wlan.isconnected():
            if time.time() - t0 > 15:
                raise RuntimeError("No se pudo conectar a la WiFi en 15s")
            time.sleep(0.5)
    print("✓ WiFi conectada, IP local:", wlan.ifconfig()[0])
    return wlan


def conectar_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((PI_IP, PI_PORT))
    print(f"✓ Conectado a la Pi en {PI_IP}:{PI_PORT}")
    return s


def leer_imu():
    """
    Reemplazá esto por la lectura real de tu segundo IMU en esta placa.
    Tiene que devolver un dict con las mismas claves que espera pi_stream.py:
    accel_x, accel_y, accel_z, roll, pitch, yaw
    """
    # --- EJEMPLO PLACEHOLDER, reemplazar por lectura real ---
    return dict(
        accel_x=0.0, accel_y=0.0, accel_z=9.8,
        roll=0.0, pitch=0.0, yaw=0.0,
    )


def main():
    conectar_wifi()
    s = conectar_socket()

    while True:
        try:
            datos = leer_imu()
            linea = json.dumps(datos) + "\n"
            s.send(linea.encode("utf-8"))
        except OSError as e:
            print("Conexión perdida, reconectando en 2s:", e)
            try:
                s.close()
            except Exception:
                pass
            time.sleep(2.0)
            try:
                s = conectar_socket()
            except OSError:
                continue

        time.sleep(INTERVALO_S)


if __name__ == "__main__":
    main()
