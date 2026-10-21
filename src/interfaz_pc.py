import sys
import json
import socket
import math
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLineEdit, QLabel, QTextEdit, QFrame)
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtCore import QThread, pyqtSignal, Qt
import pyqtgraph as pg

# --- Hilos de Comunicación ---

class DatosLiveThread(QThread):
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
                self.msleep(1000)
                
    def stop(self):
        self.corriendo = False

class ControlThread(QThread):
    log_recibido = pyqtSignal(str)
    
    def __init__(self, ip, comando):
        super().__init__()
        self.ip = ip
        self.comando = comando
        
    def run(self):
        try:
            self.log_recibido.emit(f"\n[*] Comando '{self.comando.upper()}' enviado a {self.ip}...")
            s = socket.create_connection((self.ip, 5004), timeout=10)
            s.settimeout(300.0)  # Tiempo de espera largo para no trancar al unir CSVs
            s.sendall((self.comando.upper() + "\n").encode())
            f = s.makefile("rb")
            
            while True:
                linea = f.readline()
                if not linea:
                    break
                texto = linea.decode(errors="replace").rstrip("\n")
                palabra = texto.split(" ", 1)[0]
                
                # Auto-descarga del archivo CSV
                if palabra == "CSV":
                    _, n_bytes, nombre = texto.split(" ", 2)
                    n_bytes = int(n_bytes)
                    self.log_recibido.emit(f"[*] Descargando datos ({n_bytes} bytes)...")
                    datos = f.read(n_bytes)
                    with open(nombre, "wb") as out:
                        out.write(datos)
                    self.log_recibido.emit(f"[✓] ÉXITO: Mediciones guardadas en PC -> {nombre}")
                    break
                
                self.log_recibido.emit(f"> {texto}")
                if palabra in ("LISTO", "ERROR", "FIN"):
                    break
                    
            s.close()
        except Exception as e:
            self.log_recibido.emit(f"[!] Error de conexión: {e}")

# --- Interfaz Gráfica ---

class InterfazPies(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dashboard Clínico - Captura de Movimiento IMU/Radar")
        self.resize(1400, 900)
        
        self.setStyleSheet("""
            QMainWindow { background-color: #0f172a; }
            QLabel { color: #f8fafc; font-family: 'Segoe UI', Arial; font-size: 18px; }
            QLineEdit { padding: 12px; border-radius: 6px; border: 1px solid #334155; background-color: #1e293b; color: white; font-size: 18px; font-weight: bold; }
            QPushButton { border-radius: 6px; padding: 15px; font-weight: bold; font-size: 18px; font-family: 'Segoe UI'; }
            QPushButton#btnStart { background-color: #10b981; color: white; }
            QPushButton#btnStart:hover { background-color: #059669; }
            QPushButton#btnStop { background-color: #ef4444; color: white; }
            QPushButton#btnStop:hover { background-color: #dc2626; }
            QTextEdit { background-color: #020617; color: #34d399; font-family: Consolas, monospace; border: 1px solid #334155; border-radius: 6px; padding: 12px; font-size: 14px;}
        """)
        
        pg.setConfigOptions(antialias=True)
        pg.setConfigOption('background', '#1e293b')
        pg.setConfigOption('foreground', '#f8fafc')
        
        self.max_pts = 100
        self.data_dist = np.zeros(self.max_pts)
        self.data_m_roll = np.zeros(self.max_pts)
        self.data_m_pitch = np.zeros(self.max_pts)
        self.data_p_roll = np.zeros(self.max_pts)
        self.data_p_pitch = np.zeros(self.max_pts)
        
        self.hilo_datos = None
        self.init_ui()

    def envolver_grafico(self, titulo, grafico):
        contenedor = QWidget()
        layout = QVBoxLayout(contenedor)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        lbl_titulo = QLabel(titulo)
        lbl_titulo.setFont(QFont("Segoe UI", 16, QFont.Bold))
        lbl_titulo.setAlignment(Qt.AlignCenter)
        lbl_titulo.setStyleSheet("background-color: #1e293b; padding: 10px; border-top-left-radius: 8px; border-top-right-radius: 8px;")
        
        layout.addWidget(lbl_titulo)
        layout.addWidget(grafico)
        return contenedor

    def configurar_ejes(self, plot):
        fuente_ejes = QFont("Segoe UI", 12)
        plot.getAxis("bottom").setTickFont(fuente_ejes)
        plot.getAxis("left").setTickFont(fuente_ejes)

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)
        
        left_panel = QVBoxLayout()
        left_panel.setSpacing(15)
        
        # Controles
        control_frame = QFrame()
        control_frame.setStyleSheet("QFrame { background-color: #1e293b; border-radius: 8px; border: 1px solid #334155; }")
        control_layout = QHBoxLayout(control_frame)
        
        lbl_ip = QLabel("IP Master:")
        lbl_ip.setFont(QFont("Segoe UI", 16, QFont.Bold))
        lbl_ip.setStyleSheet("border: none; font-size: 20px;")
        self.txt_ip = QLineEdit("raspberrypi.local")
        
        self.btn_start = QPushButton("▶ INICIAR MEDICIÓN")
        self.btn_start.setObjectName("btnStart")
        self.btn_stop = QPushButton("■ DETENER Y GUARDAR")
        self.btn_stop.setObjectName("btnStop")
        
        self.btn_start.clicked.connect(self.iniciar_medicion)
        self.btn_stop.clicked.connect(self.detener_medicion)
        
        control_layout.addWidget(lbl_ip)
        control_layout.addWidget(self.txt_ip)
        control_layout.addSpacing(20)
        control_layout.addWidget(self.btn_start)
        control_layout.addWidget(self.btn_stop)
        
        # Manejo seguro del Logo
        lbl_logo = QLabel()
        lbl_logo.setStyleSheet("border: none;")
        pixmap = QPixmap("ucu-3.png")
        if not pixmap.isNull():
            lbl_logo.setPixmap(pixmap.scaledToHeight(55, Qt.SmoothTransformation))
        else:
            lbl_logo.setText("UCU")
            lbl_logo.setFont(QFont("Segoe UI", 14, QFont.Bold))
            lbl_logo.setStyleSheet("color: #94a3b8; border: none;")
            
        control_layout.addStretch()
        control_layout.addWidget(lbl_logo)
        
        left_panel.addWidget(control_frame)
        
        # Piernas
        self.plot_piernas = pg.PlotWidget()
        self.configurar_ejes(self.plot_piernas)
        self.plot_piernas.setXRange(-1.2, 1.2)
        self.plot_piernas.setYRange(-1.5, 0.2)
        self.plot_piernas.hideAxis('bottom')
        self.plot_piernas.hideAxis('left')
        self.plot_piernas.showGrid(x=True, y=True, alpha=0.2)
        
        self.linea_pierna1 = self.plot_piernas.plot([0, 0], [0, -1], pen=pg.mkPen('#0ea5e9', width=10), symbol='o', symbolSize=16, symbolBrush='#0ea5e9')
        self.linea_pierna2 = self.plot_piernas.plot([0, 0], [0, -1], pen=pg.mkPen('#d946ef', width=10), symbol='o', symbolSize=16, symbolBrush='#d946ef')
        self.plot_piernas.plot([0], [0], symbol='o', symbolSize=26, symbolBrush='#f8fafc')
        
        left_panel.addWidget(self.envolver_grafico("Cinemática de Piernas (Pitch)", self.plot_piernas), stretch=3)
        
        # Distancia (AHORA SOLO PUNTOS)
        self.plot_dist = pg.PlotWidget()
        self.configurar_ejes(self.plot_dist)
        self.plot_dist.showGrid(x=True, y=True, alpha=0.2)
        # pen=None quita la línea. symbol='o' dibuja un círculo por cada medida.
        self.curva_dist = self.plot_dist.plot(pen=None, symbol='o', symbolSize=8, symbolBrush='#f59e0b', symbolPen='#d97706')
        
        left_panel.addWidget(self.envolver_grafico("Radar de Distancia (Metros)", self.plot_dist), stretch=2)
        main_layout.addLayout(left_panel, stretch=5)
        
        right_panel = QVBoxLayout()
        right_panel.setSpacing(15)
        
        # IMU 1
        self.plot_imu1 = pg.PlotWidget()
        self.configurar_ejes(self.plot_imu1)
        self.plot_imu1.showGrid(x=True, y=True, alpha=0.2)
        leyenda1 = self.plot_imu1.addLegend(offset=(10, 10))
        leyenda1.setLabelTextSize("14pt")
        self.curva_m_roll = self.plot_imu1.plot(name="Roll", pen=pg.mkPen('#3b82f6', width=3))
        self.curva_m_pitch = self.plot_imu1.plot(name="Pitch", pen=pg.mkPen('#10b981', width=3))
        
        right_panel.addWidget(self.envolver_grafico("IMU 1 (Raspberry Pi Master) - Cyan", self.plot_imu1), stretch=2)
        
        # IMU 2
        self.plot_imu2 = pg.PlotWidget()
        self.configurar_ejes(self.plot_imu2)
        self.plot_imu2.showGrid(x=True, y=True, alpha=0.2)
        leyenda2 = self.plot_imu2.addLegend(offset=(10, 10))
        leyenda2.setLabelTextSize("14pt")
        self.curva_p_roll = self.plot_imu2.plot(name="Roll", pen=pg.mkPen('#f43f5e', width=3))
        self.curva_p_pitch = self.plot_imu2.plot(name="Pitch", pen=pg.mkPen('#8b5cf6', width=3))
        
        right_panel.addWidget(self.envolver_grafico("IMU 2 (Pico Esclava) - Magenta", self.plot_imu2), stretch=2)
        
        # Logs
        lbl_logs = QLabel("Terminal de Sistema:")
        lbl_logs.setFont(QFont("Segoe UI", 16, QFont.Bold))
        self.txt_logs = QTextEdit()
        self.txt_logs.setReadOnly(True)
        right_panel.addWidget(lbl_logs)
        right_panel.addWidget(self.txt_logs, stretch=2)
        
        main_layout.addLayout(right_panel, stretch=4)
        
    def registrar_log(self, mensaje):
        self.txt_logs.append(mensaje)
        self.txt_logs.verticalScrollBar().setValue(self.txt_logs.verticalScrollBar().maximum())
        
    def iniciar_medicion(self):
        ip = self.txt_ip.text().strip()
        self.hilo_control = ControlThread(ip, "on")
        self.hilo_control.log_recibido.connect(self.registrar_log)
        self.hilo_control.start()
        
        if self.hilo_datos is None or not self.hilo_datos.corriendo:
            self.hilo_datos = DatosLiveThread(ip)
            self.hilo_datos.datos_recibidos.connect(self.actualizar_graficas)
            self.hilo_datos.start()
            
    def detener_medicion(self):
        ip = self.txt_ip.text().strip()
        self.hilo_control = ControlThread(ip, "off")
        self.hilo_control.log_recibido.connect(self.registrar_log)
        self.hilo_control.start()
        
        if self.hilo_datos is not None:
            self.hilo_datos.stop()
            self.hilo_datos = None
            
    def actualizar_graficas(self, datos):
        self.data_dist[:-1] = self.data_dist[1:]
        self.data_m_roll[:-1] = self.data_m_roll[1:]
        self.data_m_pitch[:-1] = self.data_m_pitch[1:]
        self.data_p_roll[:-1] = self.data_p_roll[1:]
        self.data_p_pitch[:-1] = self.data_p_pitch[1:]
        
        dist = datos.get("distancia_m")
        self.data_dist[-1] = dist if dist is not None else 0.0
        
        m_roll, m_pitch = datos.get("roll", 0), datos.get("pitch", 0)
        p_roll, p_pitch = datos.get("pico_roll", 0), datos.get("pico_pitch", 0)
        
        self.data_m_roll[-1] = m_roll
        self.data_m_pitch[-1] = m_pitch
        self.data_p_roll[-1] = p_roll
        self.data_p_pitch[-1] = p_pitch
        
        self.curva_dist.setData(self.data_dist)
        self.curva_m_roll.setData(self.data_m_roll)
        self.curva_m_pitch.setData(self.data_m_pitch)
        self.curva_p_roll.setData(self.data_p_roll)
        self.curva_p_pitch.setData(self.data_p_pitch)
        
        rad_m = math.radians(m_pitch)
        x1, y1 = math.sin(rad_m), -math.cos(rad_m)
        self.linea_pierna1.setData([0, x1], [0, y1])
        
        rad_p = math.radians(p_pitch)
        x2, y2 = math.sin(rad_p), -math.cos(rad_p)
        self.linea_pierna2.setData([0, x2], [0, y2])

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ventana = InterfazPies()
    ventana.show()
    sys.exit(app.exec_())