# Corre en la Raspberry Pi Pico W con CircuitPython. Se llama code.py para que arranque
# sola al alimentarla. Copiar al disco CIRCUITPY: code.py, config.py, imu_pico.py y, en
# lib/, adafruit_bno08x y adafruit_bus_device (circup install adafruit_bno08x).
#
# Lee la IMU a FRECUENCIA_HZ y manda cada muestra a la Zero (pico_receptor.py) por TCP,
# una línea JSON por muestra:
#   - las muestras quedan en un buffer (BUFFER_MAX) hasta que la Zero confirma que las
#     guardó ("ack <seq>"). Si la conexión se corta, al reconectar se reenvía todo lo no
#     confirmado; la Zero descarta los repetidos. Si el corte dura más de lo que entra en
#     el buffer, se descartan las más viejas (la Zero lo ve como huecos en seq).
#   - el socket se reintenta cada REINTENTO_MS sin frenar el muestreo; la WiFi en cambio
#     sí lo frena mientras intenta conectarse (ver WIFI_TIMEOUT_S en config.py).
# Cada muestra lleva seq (para detectar pérdidas y descartar reenvíos) y t_pico_ms (ms desde
# el arranque, para que la Zero la ubique en su reloj).
#
# LED de la placa: apagado = sin WiFi, parpadeo = WiFi sin conexión a la Zero,
# fijo = mandando datos.

import errno
import gc
import json
import os
import time

import board
import digitalio
import microcontroller
import socketpool
import wifi

import config
from imu_pico import crear_imu, ms

REINTENTO_MS = 2000       # cada cuánto reintentar el socket si hay WiFi pero no Zero
TIMEOUT_SOCKET_S = 0.5    # máximo que una escritura puede frenar el muestreo
MAX_LINEAS_POR_ENVIO = 40
ESTADO_MS = 5000          # cada cuánto imprimir estado por USB
SIN_DATOS = (errno.EAGAIN, errno.ETIMEDOUT)


class Enlace:
    def __init__(self, boot_id, imu_tipo):
        self.hola = {
            "tipo": "hola", "boot_id": boot_id, "imu": imu_tipo, "hz": config.FRECUENCIA_HZ,
        }
        self.pool = None
        self.sock = None
        self.t_intento = ms() - REINTENTO_MS
        self.t_intento_wifi = ms() - config.WIFI_REINTENTO_S * 1000
        self.rx = b""
        self.buf = bytearray(256)
        self.enviadas = 0   # cuántas de las pendientes ya salieron por esta conexión
        self.conexiones = 0
        try:
            # Sin ahorro de energía: con el modo por defecto la Pico W agrupa los envíos
            # y aparecen demoras de cientos de ms.
            wifi.radio.power_management = wifi.PowerManagement.NONE
        except Exception:
            pass  # versiones de CircuitPython que no lo tienen

    def wifi_ok(self):
        return wifi.radio.ipv4_address is not None

    def _conectar_wifi(self, wdt):
        if ms() - self.t_intento_wifi < config.WIFI_REINTENTO_S * 1000:
            return False
        self.t_intento_wifi = ms()
        print("Conectando a WiFi:", config.SSID)
        if wdt is not None:
            wdt.feed()
        try:
            wifi.radio.connect(config.SSID, config.PASSWORD, timeout=config.WIFI_TIMEOUT_S)
        except Exception as e:
            print("WiFi: no conectó (%s)" % e)
            return False
        print("✓ WiFi conectada, IP local:", wifi.radio.ipv4_address)
        return True

    def mantener(self, wdt=None):
        """Reintenta WiFi y socket. True si hay socket."""
        if self.sock is not None:
            return True
        if not self.wifi_ok() and not self._conectar_wifi(wdt):
            return False
        if ms() - self.t_intento < REINTENTO_MS:
            return False
        self.t_intento = ms()

        if self.pool is None:
            self.pool = socketpool.SocketPool(wifi.radio)
        s = self.pool.socket(self.pool.AF_INET, self.pool.SOCK_STREAM)
        try:
            s.settimeout(TIMEOUT_SOCKET_S * 4)
            s.connect((config.MASTER_IP, config.MASTER_PORT))
            s.settimeout(TIMEOUT_SOCKET_S)
            # El hola lleva el reloj actual: le da a la Zero una referencia fresca para
            # ubicar las muestras que se acumularon mientras no había conexión.
            self.hola["t_pico_ms"] = ms()
            self._escribir(s, (json.dumps(self.hola) + "\n").encode())
        except OSError as e:
            print("No pude conectar a la Zero %s:%d (%s)"
                  % (config.MASTER_IP, config.MASTER_PORT, e))
            s.close()
            return False
        self.sock = s
        self.rx = b""
        self.enviadas = 0
        self.conexiones += 1
        print("✓ Conectada a la Zero")
        return True

    @staticmethod
    def _escribir(s, datos):
        # send() puede mandar solo una parte: se insiste hasta mandar todo o hasta timeout.
        mv = memoryview(datos)
        while len(mv):
            n = s.send(mv)
            if n <= 0:
                raise OSError("send devolvió %d" % n)
            mv = mv[n:]

    def enviar(self, pendientes):
        """Manda hasta MAX_LINEAS_POR_ENVIO de las pendientes que todavía no salieron."""
        lote = pendientes[self.enviadas:self.enviadas + MAX_LINEAS_POR_ENVIO]
        if not lote:
            return
        try:
            self._escribir(self.sock, b"".join(linea for _, linea in lote))
            self.enviadas += len(lote)
        except OSError as e:
            # Si la escritura quedó a medias, la Zero descarta la línea cortada y las
            # repetidas (por seq), así que al reconectar se reenvía todo lo no confirmado.
            print("Conexión perdida con la Zero:", e)
            self.cerrar()

    def leer_ack(self):
        """Último seq confirmado por la Zero, o None si no llegó nada nuevo. No bloquea."""
        try:
            self.sock.settimeout(0)
            n = self.sock.recv_into(self.buf)
        except OSError as e:
            if e.errno in SIN_DATOS:
                self.sock.settimeout(TIMEOUT_SOCKET_S)
                return None
            print("Conexión perdida con la Zero:", e)
            self.cerrar()
            return None
        self.sock.settimeout(TIMEOUT_SOCKET_S)
        if n == 0:
            print("La Zero cerró la conexión")
            self.cerrar()
            return None
        lineas = (self.rx + bytes(self.buf[:n])).split(b"\n")
        self.rx = lineas.pop()  # resto incompleto, se completa en la próxima lectura
        ack = None
        for linea in lineas:
            if linea.startswith(b"ack "):
                ack = int(linea[4:].decode())
        return ack

    def cerrar(self):
        try:
            self.sock.close()
        except Exception:
            pass
        self.sock = None
        self.enviadas = 0


def crear_wdt():
    if not config.USAR_WDT:
        return None
    from watchdog import WatchDogMode
    wdt = microcontroller.watchdog
    wdt.timeout = 8
    wdt.mode = WatchDogMode.RESET
    return wdt


def main():
    wdt = crear_wdt()
    try:
        led = digitalio.DigitalInOut(board.LED)
        led.direction = digitalio.Direction.OUTPUT
    except Exception:
        led = None

    boot_id = "".join("%02x" % b for b in os.urandom(4))
    imu = crear_imu()
    enlace = Enlace(boot_id, imu.TIPO)
    print("boot_id:", boot_id, "| IMU:", imu.TIPO, "|", config.FRECUENCIA_HZ, "Hz")

    periodo = 1000 // config.FRECUENCIA_HZ
    proximo = ms()
    t_estado = proximo
    seq = 0
    pendientes = []    # (seq, línea) no confirmadas por la Zero
    descartadas = 0
    lazo_max = 0       # ms del lazo más lento desde el último estado (si > periodo, no da)

    while True:
        espera = proximo - ms()
        if espera > 0:
            time.sleep(espera / 1000)
        elif espera < -5 * periodo:
            proximo = ms()  # nos atrasamos mucho: no intentar recuperar de golpe
        proximo += periodo
        t_lazo = ms()

        # --- Muestra ---
        d = imu.leer()
        d["seq"] = seq
        d["t_pico_ms"] = t_lazo
        seq += 1
        pendientes.append((d["seq"], (json.dumps(d) + "\n").encode()))
        if len(pendientes) > config.BUFFER_MAX:
            pendientes.pop(0)
            descartadas += 1
            if enlace.enviadas:
                enlace.enviadas -= 1

        # --- Red ---
        if enlace.mantener(wdt):
            ack = enlace.leer_ack()
            if ack is not None:
                k = 0
                while k < len(pendientes) and pendientes[k][0] <= ack:
                    k += 1
                del pendientes[:k]
                enlace.enviadas = max(0, enlace.enviadas - k)
            if enlace.sock is not None:
                enlace.enviar(pendientes)

        if led is not None:
            if enlace.sock is not None:
                led.value = True
            elif enlace.wifi_ok():
                led.value = not led.value
            else:
                led.value = False

        if wdt is not None:
            wdt.feed()
        if seq % 20 == 0:
            gc.collect()  # GC chico y frecuente en vez de uno largo en un momento cualquiera

        lazo_max = max(lazo_max, ms() - t_lazo)
        if ms() - t_estado >= ESTADO_MS:
            t_estado = ms()
            print("seq %d | pendientes %d | descartadas %d | wifi %s | zero %s | "
                  "lazo max %d ms | imu reinicios %d | ram libre %d"
                  % (seq, len(pendientes), descartadas, enlace.wifi_ok(),
                     enlace.sock is not None, lazo_max, imu.reinicios, gc.mem_free()))
            lazo_max = 0


try:
    main()
except KeyboardInterrupt:
    print("Detenido")
except Exception as e:
    # Sin nadie mirando (en la caminata), mejor reiniciar que quedar muerta.
    try:
        import traceback
        traceback.print_exception(e)
    except Exception:
        print(repr(e))
    time.sleep(5)
    microcontroller.reset()
