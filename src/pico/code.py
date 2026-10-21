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

REINTENTO_MS = 2000       
TIMEOUT_SOCKET_S = 0.5    
MAX_LINEAS_POR_ENVIO = 40
ESTADO_MS = 5000          
HB_MS = 250               
PREP_MAX_MS = 3000        
                          

REPOSO, LISTO, MIDIENDO, CERRANDO = "reposo", "listo", "midiendo", "cerrando"
SIN_DATOS = (errno.EAGAIN, errno.ETIMEDOUT)
BALIZA_PUERTO = 5007      # la master avisa su IP por UDP (ver pico_receptor.py)
BALIZA_VIGENCIA_MS = 10000


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
        self.cmds = []      # órdenes de la master recibidas y todavía no atendidas
        self.udp = None
        self.ubuf = bytearray(64)
        self.baliza_ip = None
        self.baliza_puerto = config.MASTER_PORT
        self.t_baliza = 0
        self.t_aviso = 0
        self.conexiones = 0
        try:
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

    def _escuchar_baliza(self):
        """Anota la IP de quien manda "MASTER <puerto>" por UDP. No bloquea."""
        try:
            if self.udp is None:
                if self.pool is None:
                    self.pool = socketpool.SocketPool(wifi.radio)
                u = self.pool.socket(self.pool.AF_INET, self.pool.SOCK_DGRAM)
                u.bind(("0.0.0.0", BALIZA_PUERTO))
                u.settimeout(0)
                self.udp = u
            for _ in range(4):
                n, addr = self.udp.recvfrom_into(self.ubuf)
                texto = bytes(self.ubuf[:n]).decode()
                if texto.startswith("MASTER "):
                    if self.baliza_ip != addr[0]:
                        print("Master encontrada en", addr[0])
                    self.baliza_ip = addr[0]
                    self.baliza_puerto = int(texto[7:])
                    self.t_baliza = ms()
        except OSError as e:
            if e.errno not in SIN_DATOS:
                try:
                    self.udp.close()
                except Exception:
                    pass
                self.udp = None     # se recrea (p. ej. si la WiFi se reconectó)
        except (ValueError, UnicodeError):
            pass

    def _destino(self):
        """IP y puerto de la master: la de la baliza si es reciente; si no, MASTER_IP del config."""
        if self.baliza_ip is not None and ms() - self.t_baliza < BALIZA_VIGENCIA_MS:
            return self.baliza_ip, self.baliza_puerto
        ip = getattr(config, "MASTER_IP", None)
        if ip and ip != "auto":
            return ip, config.MASTER_PORT
        return None, None

    def mantener(self, wdt=None):
        """Reintenta WiFi y socket. True si hay socket."""
        if self.sock is not None:
            return True
        if not self.wifi_ok() and not self._conectar_wifi(wdt):
            return False
        self._escuchar_baliza()
        if ms() - self.t_intento < REINTENTO_MS:
            return False
        self.t_intento = ms()
        ip, puerto = self._destino()
        if ip is None:
            if ms() - self.t_aviso > 5000:
                self.t_aviso = ms()
                print("Esperando el aviso de la master (¿está corriendo master.py?)")
            return False

        if self.pool is None:
            self.pool = socketpool.SocketPool(wifi.radio)
        s = self.pool.socket(self.pool.AF_INET, self.pool.SOCK_STREAM)
        try:
            s.settimeout(TIMEOUT_SOCKET_S * 4)
            s.connect((ip, puerto))
            s.settimeout(TIMEOUT_SOCKET_S)
            self.hola["t_pico_ms"] = ms()
            self._escribir(s, (json.dumps(self.hola) + "\n").encode())
        except OSError as e:
            print("No pude conectar a la Zero %s:%d (%s)" % (ip, puerto, e))
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

    def enviar_json(self, obj):
        """Manda un mensaje suelto (latido, listo, fin). False si se perdió la conexión."""
        try:
            self._escribir(self.sock, (json.dumps(obj) + "\n").encode())
            return True
        except OSError as e:
            print("Conexión perdida con la Zero:", e)
            self.cerrar()
            return False

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
            elif linea.strip():
                try:
                    self.cmds.append(linea.decode().strip())
                except ValueError:
                    pass
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
    estado = REPOSO
    sid = ""
    proximo = ms()
    t_estado = t_hb = t_gc = proximo
    seq = 0
    pendientes = []    # (seq, línea) no confirmadas por la Zero
    descartadas = 0
    lazo_max = 0       # ms del lazo más lento desde el último estado (si > periodo, no da)
    reinicios_base = imu.reinicios
    listo_msg = None   # último "listo" (se reenvía si la master repite 'prep')
    ultimo_fin = None  # último "fin" (se reenvía si la master repite 'stop')

    while True:
        # --- Espera: reposo = 20 ms; midiendo = hasta la próxima muestra (máx. 20 ms) ---
        ahora = ms()
        if estado == MIDIENDO:
            if ahora < proximo:
                time.sleep(min(proximo - ahora, 20) / 1000)
        else:
            time.sleep(0.02)

        # --- Muestra: solo midiendo y a partir de t0 ---
        t_lazo = ms()
        tomo = estado == MIDIENDO and t_lazo >= proximo
        if tomo:
            if t_lazo - proximo > 5 * periodo:
                proximo = t_lazo  # nos atrasamos mucho: no intentar recuperar de golpe
            proximo += periodo
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
            if enlace.sock is not None and ms() - t_hb >= HB_MS:
                t_hb = ms()
                enlace.enviar_json({"tipo": "hb", "t_pico_ms": t_hb})

        # --- Órdenes de la master ---
        cmds, enlace.cmds = enlace.cmds, []
        for c in cmds:
            p = c.split()
            if not p:
                continue
            if p[0] == "prep" and len(p) >= 2:
                if estado == LISTO and p[1] == sid and listo_msg is not None:
                    enlace.enviar_json(listo_msg)      # 'prep' repetido: solo confirmar de nuevo
                elif estado in (REPOSO, LISTO):
                    sid = p[1]
                    pendientes = []
                    enlace.enviadas = 0
                    seq = 0
                    descartadas = 0
                    ultimo_fin = None
                    if hasattr(imu, "reiniciar"):
                        imu.reiniciar("preparar")
                    ok = 0
                    t_prep = ms()
                    while ms() - t_prep < PREP_MAX_MS:
                        if wdt is not None:
                            wdt.feed()
                        if imu.leer()["imu_ok"]:
                            ok = 1
                            break
                        time.sleep(0.1)
                    reinicios_base = imu.reinicios
                    listo_msg = {"tipo": "listo", "sid": sid, "imu_ok": ok, "imu": imu.TIPO,
                                 "t_pico_ms": ms()}
                    estado = LISTO
                    print("prep", sid, "| IMU ok:", ok)
                    enlace.enviar_json(listo_msg)
                else:
                    print("prep ignorado, estado:", estado)
            elif p[0] == "start" and len(p) >= 2:
                if estado == LISTO:
                    try:
                        t0 = int(p[1])
                    except ValueError:
                        continue
                    proximo = t0
                    estado = MIDIENDO
                    print("start t0=%d (faltan %d ms)" % (t0, t0 - ms()))
            elif p[0] == "stop":
                if estado in (LISTO, MIDIENDO):
                    estado = CERRANDO
                elif estado == REPOSO and ultimo_fin is not None:
                    enlace.enviar_json(ultimo_fin)     # 'stop' repetido: confirmar de nuevo

        # --- Cierre: 'fin' recién cuando la master confirmó TODAS las muestras ---
        if estado == CERRANDO and not pendientes and enlace.sock is not None:
            fin = {"tipo": "fin", "sid": sid, "ultimo_seq": seq - 1, "descartadas": descartadas,
                   "imu_reinicios": imu.reinicios - reinicios_base}
            if enlace.enviar_json(fin):
                ultimo_fin = fin
                estado = REPOSO
                print("fin", sid, "| muestras:", seq, "| descartadas:", descartadas)

        if led is not None:
            if enlace.sock is not None:
                led.value = True
            elif enlace.wifi_ok():
                led.value = not led.value
            else:
                led.value = False

        if wdt is not None:
            wdt.feed()
        if tomo:
            if seq % 20 == 0:
                gc.collect()  # GC chico y frecuente en vez de uno largo en un momento cualquiera
            lazo_max = max(lazo_max, ms() - t_lazo)
        elif estado != MIDIENDO and ms() - t_gc >= 1000:
            t_gc = ms()
            gc.collect()
        if ms() - t_estado >= ESTADO_MS:
            t_estado = ms()
            print("%s | seq %d | pendientes %d | descartadas %d | wifi %s | zero %s | "
                  "lazo max %d ms | imu reinicios %d | ram libre %d"
                  % (estado, seq, len(pendientes), descartadas, enlace.wifi_ok(),
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
