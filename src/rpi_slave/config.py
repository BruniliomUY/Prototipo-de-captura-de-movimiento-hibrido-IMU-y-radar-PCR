# Configuración de la Pico W (CircuitPython). Es lo único que debería hacer falta tocar.

# --- Red: la misma WiFi a la que está conectada la Zero (o el hotspot de la Zero) ---
# No uses CIRCUITPY_WIFI_SSID en settings.toml: eso activa además el web workflow
# (servidor web en la Pico) y le saca RAM y tiempo al muestreo.
SSID = "UCU_"
PASSWORD = "raspberrypi"

# IP de la Zero en esa red. Verificala en la Zero con:  hostname -I
#   - hotspot con hostapd/dnsmasq: suele ser 192.168.4.1
#   - hotspot con NetworkManager (Bookworm, nmcli ... hotspot): suele ser 10.42.0.1
#   - router externo: la que le dé el router (conviene reservarla por MAC)
MASTER_IP = "10.21.200.143"
MASTER_PORT = 5006          # pico_receptor.py en la Zero

# En CircuitPython, wifi.radio.connect() BLOQUEA: mientras intenta conectarse no se mide.
# Por eso se intenta poco y con timeout corto; las muestras de entre intentos van al buffer.
WIFI_TIMEOUT_S = 5
WIFI_REINTENTO_S = 10

# --- Muestreo ---
FRECUENCIA_HZ = 20          # igual al update_rate del radar en pi_master.py

# Muestras guardadas hasta que la Zero confirma que las recibió.
# Cada una ocupa ~150 B de RAM: 200 = ~30 KB = 10 s de corte a 20 Hz.
# CircuitPython + adafruit_bno08x dejan poca RAM libre: mirá "ram libre" en el estado
# antes de subirlo.
BUFFER_MAX = 200

# --- IMU ---
# "bno085": la IMU (adafruit_bno08x)
IMU = "sim"

# BNO085 por I2C. Pines de la Pico W: GP4 = pin 6 (SDA), GP5 = pin 7 (SCL), 3V3 = pin 36.
I2C_SDA = "GP4"
I2C_SCL = "GP5"
I2C_FREQ = 400_000
BNO_ADDR = 0x4A             # 0x4B si el breakout tiene el jumper DI/ADR puenteado
BNO_RESET_PIN = None        # p. ej. "GP6" si cableás el RST del BNO085: el reinicio
                            # por hardware recupera el sensor mucho mejor que el de software

# --- Robustez para usarla sin cable ---
# Watchdog: si el programa se cuelga, la Pico se reinicia sola a los 8 s.
# Dejalo en False mientras desarrollás (si no, se reinicia al frenarlo con Ctrl-C).
USAR_WDT = False

