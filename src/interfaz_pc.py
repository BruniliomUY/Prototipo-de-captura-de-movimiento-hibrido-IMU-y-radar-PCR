import sys
import json
import socket
import math
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLineEdit, QLabel, QTextEdit)
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import QThread, pyqtSignal, Qt
import pyqtgraph as pg

# --- Hilos de Comunicación ---

class DatosLiveThread(QThread):
    """Hilo para conectarse al puerto 5005 y recibir el JSON en tiempo real."""
    datos_recibidos = pyqtSignal(dict)
    
    def __init__(self, ip):
        super().__init__()
        self.ip = ip
        self.corriendo = True
        
    def run(self):
        while self.corriendo:
            try:
                with socket.create_connection((self.ip, 5005), timeout=2) as s:
                    f = s.makefile("r")
                    while self.corriendo:
                        linea = f.readline()
                        if not linea:
                            break
                        try:
                            datos = json.loads(linea)
                            self.datos_recibidos.emit(datos)
                        except json.JSONDecodeError:
                            pass
            except OSError:
                self.msleep(1000) # Reintenta cada 1 segundo si falla
                
    def stop(self):
        self.corriendo = False

class ControlThread(QThread):
    """Hilo para enviar comandos (ON/OFF) al puerto 5004 y leer la respuesta (logs)."""
    log_recibido = pyqtSignal(str)
    
    def __init__(self, ip, comando):
        super().__init__()
        self.ip = ip
        self.comando = comando
        
    def run(self):
        try:
            self.log_recibido.emit(f"[*] Enviando comando '{self.comando.upper()}' a {self.ip}...")
            s = socket.create_connection((self.ip, 5004), timeout=10)
            s.sendall((self.comando.upper() + "\n").encode())
            f = s.makefile("rb")
            while True:
                linea = f.readline()
                if not linea:
                    self.log_recibido.emit("[-] Conexión de control cerrada por la master.")
                    break
                texto = linea.decode(errors="replace").rstrip("\n")
                self.log_recibido.emit(f"> {texto}")
                if texto.split(" ", 1)[0] in ("LISTO", "FIN", "ERROR", "CSV"):
                    break
            s.close()
        except Exception as e:
            self.log_recibido.emit(f"[!] Error de conexión: {e}")

# --- Interfaz Gráfica ---

class InterfazPies(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sistema de Medición de Pies - Master & Pico")
        self.resize(1200, 800)
        
        # Buffers de datos para las gráficas (últimos 100 puntos)
        self.max_pts = 100
        self.data_dist = np.zeros(self.max_pts)
        self.data_m_roll = np.zeros(self.max_pts)
        self.data_m_pitch = np.zeros(self.max_pts)
        self.data_p_roll = np.zeros(self.max_pts)
        self.data_p_pitch = np.zeros(self.max_pts)
        
        self.hilo_datos = None
        self.init_ui()
        
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Estilos generales pyqtgraph
        pg.setConfigOptions(antialias=True)
        pg.setConfigOption('background', '#1e1e1e')
        pg.setConfigOption('foreground', 'w')
        
        # ================= PANEL IZQUIERDO =================
        left_panel = QVBoxLayout()
        
        # 1. Controles (Start, Stop, IP) y Logo
        control_layout = QHBoxLayout()
        
        control_inputs = QVBoxLayout()
        ip_layout = QHBoxLayout()
        lbl_ip = QLabel("IP Master:")
        lbl_ip.setFont(QFont("Arial", 10, QFont.Bold))
        self.txt_ip = QLineEdit("raspberrypi.local")
        ip_layout.addWidget(lbl_ip)
        ip_layout.addWidget(self.txt_ip)
        
        btn_layout = QHBoxLayout()
        self.btn_start = QPushButton("START")
        self.btn_start.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold; padding: 10px;")
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setStyleSheet("background-color: #C62828; color: white; font-weight: bold; padding: 10px;")
        
        self.btn_start.clicked.connect(self.iniciar_medicion)
        self.btn_stop.clicked.connect(self.detener_medicion)
        
        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_stop)
        
        control_inputs.addLayout(ip_layout)
        control_inputs.addLayout(btn_layout)
        
        # Logo UCU
        lbl_logo = QLabel()
        try:
            pixmap = QPixmap("ucu-3.png")
            lbl_logo.setPixmap(pixmap.scaledToHeight(60, Qt.SmoothTransformation))
        except:
            lbl_logo.setText("[Logo UCU]")
            
        control_layout.addLayout(control_inputs)
        control_layout.addStretch()
        control_layout.addWidget(lbl_logo)
        
        left_panel.addLayout(control_layout)
        
        # 2. Modelo de piernas (Gráfico X-Y)
        self.plot_piernas = pg.PlotWidget(title="Modelo de Piernas (Pitch)")
        self.plot_piernas.setXRange(-1, 1)
        self.plot_piernas.setYRange(-1.2, 0.2)
        self.plot_piernas.hideAxis('bottom')
        self.plot_piernas.hideAxis('left')
        self.linea_pierna1 = self.plot_piernas.plot([0, 0], [0, -1], pen=pg.mkPen('c', width=5))
        self.linea_pierna2 = self.plot_piernas.plot([0, 0], [0, -1], pen=pg.mkPen('m', width=5))
        left_panel.addWidget(self.plot_piernas)
        
        # 3. Distancia
        self.plot_dist = pg.PlotWidget(title="Distancia (Radar)")
        self.plot_dist.setLabel('left', 'Metros')
        self.curva_dist = self.plot_dist.plot(pen=pg.mkPen('y', width=2))
        left_panel.addWidget(self.plot_dist)
        
        main_layout.addLayout(left_panel, stretch=1)
        
        # ================= PANEL DERECHO =================
        right_panel = QVBoxLayout()
        
        # 1. IMU 1 (Master)
        self.plot_imu1 = pg.PlotWidget(title="IMU 1 (Master Pi)")
        self.plot_imu1.addLegend()
        self.curva_m_roll = self.plot_imu1.plot(name="Roll", pen=pg.mkPen('r', width=2))
        self.curva_m_pitch = self.plot_imu1.plot(name="Pitch", pen=pg.mkPen('g', width=2))
        right_panel.addWidget(self.plot_imu1)
        
        # 2. IMU 2 (Pico)
        self.plot_imu2 = pg.PlotWidget(title="IMU 2 (Pico)")
        self.plot_imu2.addLegend()
        self.curva_p_roll = self.plot_imu2.plot(name="Roll", pen=pg.mkPen('r', width=2))
        self.curva_p_pitch = self.plot_imu2.plot(name="Pitch", pen=pg.mkPen('g', width=2))
        right_panel.addWidget(self.plot_imu2)
        
        # 3. Logs
        lbl_logs = QLabel("Logs:")
        lbl_logs.setFont(QFont("Arial", 10, QFont.Bold))
        self.txt_logs = QTextEdit()
        self.txt_logs.setReadOnly(True)
        self.txt_logs.setStyleSheet("background-color: #000; color: #0f0; font-family: monospace;")
        right_panel.addWidget(lbl_logs)
        right_panel.addWidget(self.txt_logs)
        
        main_layout.addLayout(right_panel, stretch=1)
        
    def registrar_log(self, mensaje):
        self.txt_logs.append(mensaje)
        self.txt_logs.verticalScrollBar().setValue(self.txt_logs.verticalScrollBar().maximum())
        
    def iniciar_medicion(self):
        ip = self.txt_ip.text().strip()
        self.txt_logs.clear()
        
        # Lanzar comando ON
        self.hilo_control = ControlThread(ip, "on")
        self.hilo_control.log_recibido.connect(self.registrar_log)
        self.hilo_control.start()
        
        # Iniciar lectura de datos en vivo (si no está corriendo ya)
        if self.hilo_datos is None or not self.hilo_datos.corriendo:
            self.hilo_datos = DatosLiveThread(ip)
            self.hilo_datos.datos_recibidos.connect(self.actualizar_graficas)
            self.hilo_datos.start()
            
    def detener_medicion(self):
        ip = self.txt_ip.text().strip()
        # Lanzar comando OFF
        self.hilo_control = ControlThread(ip, "off")
        self.hilo_control.log_recibido.connect(self.registrar_log)
        self.hilo_control.start()
        
        # Detener actualización gráfica
        if self.hilo_datos is not None:
            self.hilo_datos.stop()
            self.hilo_datos = None
            
    def actualizar_graficas(self, datos):
        # Desplazar arrays hacia la izquierda
        self.data_dist[:-1] = self.data_dist[1:]
        self.data_m_roll[:-1] = self.data_m_roll[1:]
        self.data_m_pitch[:-1] = self.data_m_pitch[1:]
        self.data_p_roll[:-1] = self.data_p_roll[1:]
        self.data_p_pitch[:-1] = self.data_p_pitch[1:]
        
        # Insertar nuevo dato
        dist = datos.get("distancia_m")
        self.data_dist[-1] = dist if dist is not None else 0.0
        
        m_roll, m_pitch = datos.get("roll", 0), datos.get("pitch", 0)
        p_roll, p_pitch = datos.get("pico_roll", 0), datos.get("pico_pitch", 0)
        
        self.data_m_roll[-1] = m_roll
        self.data_m_pitch[-1] = m_pitch
        self.data_p_roll[-1] = p_roll
        self.data_p_pitch[-1] = p_pitch
        
        # Actualizar curvas (IMUs y Distancia)
        self.curva_dist.setData(self.data_dist)
        self.curva_m_roll.setData(self.data_m_roll)
        self.curva_m_pitch.setData(self.data_m_pitch)
        self.curva_p_roll.setData(self.data_p_roll)
        self.curva_p_pitch.setData(self.data_p_pitch)
        
        # Actualizar Modelo de Piernas (usando trigonometría simple con el Pitch)
        # Pierna 1 (Cyan) usa IMU Master
        rad_m = math.radians(m_pitch)
        x1, y1 = math.sin(rad_m), -math.cos(rad_m)
        self.linea_pierna1.setData([0, x1], [0, y1])
        
        # Pierna 2 (Magenta) usa IMU Pico
        rad_p = math.radians(p_pitch)
        x2, y2 = math.sin(rad_p), -math.cos(rad_p)
        self.linea_pierna2.setData([0, x2], [0, y2])

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ventana = InterfazPies()
    ventana.show()
    sys.exit(app.exec_())