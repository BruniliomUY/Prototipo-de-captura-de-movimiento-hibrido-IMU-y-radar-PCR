# Prototipo-de-captura-de-movimiento-hibrido-IMU-y-radar-PCR-.
Prueba de concepto de dispositivo movil de captura de datos de movimiento y distancia a partir de una IMU(BNO08x) y un PCRradar XM125.

| Dispositivo | Placa | Sensores | Rol |
|---|---|---|---|
| Master | Raspberry Pi Zero | Radar XM125 + IMU BNO085 | Mide, centraliza y guarda todo |
| Satélite | Raspberry Pi Pico W | IMU | Mide y le manda los datos al master por WiFi |
