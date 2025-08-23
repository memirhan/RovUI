#!/usr/bin/env python3

import cv2
import numpy as np
import serial
import time
import sys
import time
from dataclasses import dataclass
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QThread, pyqtSignal
import os
import sqlite3


class VideoWidget(QtWidgets.QLabel):
    frameUpdated = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(640, 360)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self._playing = True
        self._rec = True
        self._frame = None

    def setFrame(self, frame: np.ndarray):
        self._frame = frame
        if frame is None:
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        qimg = QtGui.QImage(rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qimg)
        # Fit to label keeping aspect ratio
        self.setPixmap(pix.scaled(self.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        self.frameUpdated.emit()

    def togglePlay(self):
        self._playing = not self._playing
        self.update()

    def isPlaying(self):
        return self._playing

    def setPlaying(self, playing: bool):
        self._playing = playing
        self.update()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Force redraw scaling
        if self._frame is not None:
            self.setFrame(self._frame)

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.end()


class PusulaWidget(QtWidgets.QWidget):
    yawChanged = QtCore.pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._yaw = 0.0  # degrees 0-360
        self.setMinimumSize(220, 220)

    def setYaw(self, yaw: float):
        self._yaw = float(yaw) % 360.0
        self.yawChanged.emit(self._yaw)
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = self.rect().adjusted(8, 8, -8, -8)
        center = rect.center()
        radius = min(rect.width(), rect.height()) // 2

        # Arka plan
        painter.setPen(QtGui.QPen(QtGui.QColor(60, 60, 60), 2))
        painter.setBrush(QtGui.QColor(20, 20, 20))
        painter.drawEllipse(center, radius, radius)

        # Ticks
        painter.setPen(QtGui.QPen(QtGui.QColor(200, 200, 200), 2))
        for deg in range(0, 360, 10):
            angle = np.deg2rad(deg)
            l = 10 if deg % 30 else 16
            x1 = center.x() + int((radius - 8) * np.cos(angle))
            y1 = center.y() + int((radius - 8) * np.sin(angle))
            x2 = center.x() + int((radius - 8 - l) * np.cos(angle))
            y2 = center.y() + int((radius - 8 - l) * np.sin(angle))
            painter.drawLine(x1, y1, x2, y2)

        # Kuzey iÅareti
        painter.setPen(QtGui.QPen(QtGui.QColor(180, 180, 180), 1))
        painter.setBrush(QtGui.QColor(70, 70, 70))
        painter.drawEllipse(center, 6, 6)

        # Ä°bre (yaw)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(100, 200, 255))
        angle = np.deg2rad(self._yaw - 90)
        x = center.x() + int((radius - 24) * np.cos(angle))
        y = center.y() + int((radius - 24) * np.sin(angle))
        path = QtGui.QPainterPath()
        path.moveTo(center)
        path.lineTo(x, y)
        painter.drawPath(path)

        # Merkez artÄ±
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 180), 2))
        painter.drawLine(center.x() - 12, center.y(), center.x() + 12, center.y())
        painter.drawLine(center.x(), center.y() - 12, center.x(), center.y() + 12)

        # YazÄ±: derece
        painter.setPen(QtGui.QColor(220, 220, 220))
        painter.setFont(QtGui.QFont("Oswald", 12))
        painter.drawText(self.rect(), QtCore.Qt.AlignBottom | QtCore.Qt.AlignHCenter, f"{self._yaw:.1f}Â°")
        painter.end()


@dataclass
class Telemetri:
    mode: str = "Thrust"
    yaw: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    depth_m: float = 0.0
    humidity: float = 17.0
    temperature: float = 27.0
    arm: bool = False
    x_axis: float = 0.0
    y_axis: float = 0.0
    z_axis: float = 0.0

class JoystickWidget(QtWidgets.QWidget):
    def __init__(self, parent=None, lock_x=False, lock_y=False):
        super().__init__(parent)
        self.setFixedSize(200, 200)
        self.pos_x = 512
        self.pos_y = 512
        self.lock_x = lock_x
        self.lock_y = lock_y

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        rect = self.rect()

        # DÄ±Å kare
        painter.setPen(QtGui.QPen(QtGui.QColor("black"), 2))
        painter.drawRect(rect.adjusted(1, 1, -1, -1))

        # X ve Y eksenleri
        center_px = rect.width() / 2
        center_py = rect.height() / 2
        painter.setPen(QtGui.QPen(QtGui.QColor("gray"), 1, QtCore.Qt.DashLine))
        painter.drawLine(int(center_px), 0, int(center_px), rect.height())
        painter.drawLine(0, int(center_py), rect.width(), int(center_py))

        # Joystick topu
        painter.setBrush(QtGui.QColor("blue"))
        painter.setPen(QtCore.Qt.NoPen)

        scale_x = rect.width() / 1024
        scale_y = rect.height() / 1024
        px = int(self.pos_x * scale_x)
        py = int(self.pos_y * scale_y)

        painter.drawEllipse(QtCore.QPointF(px, py), 10, 10)

    def update_position(self, x, y):
        """Analog veriyi gÃ¼ncelle"""
        if not self.lock_x:
            self.pos_x = max(0, min(1024, x))
        if not self.lock_y:
            self.pos_y = max(0, min(1024, y))
        self.update()
        
class SerialReaderThread(QThread):
    # Gelen veriyi GUI'ye iletmek icin sinyal
    dataReceived = pyqtSignal(str)
    def __init__(self, port="/dev/ttyUSB0", baudrate=9600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self._running = True

    def run(self):
        ser = serial.Serial(self.port, self.baudrate, timeout=1)
        time.sleep(2) # Bağlantının oturması için bekle

        try:
            while self._running:
                if ser.in_waiting > 0:  # Veri varsa
                    data = ser.readline().decode('utf-8').strip()  # Veri oku
                    self.dataReceived.emit(data)  # GUI'ye veri gdnder
        except KeyboardInterrupt:
            print("Terminating Connection")
        finally:
            ser.close()

    def stop(self):
        self._running = False  # Thread'i durdur
        self.wait()


# ----------------------------- Ana Pencere -----------------------------
class ROVKonsol(QtWidgets.QMainWindow):
    def __init__(self, kameraIndex=0):
        super().__init__()
        self.setWindowTitle("ROV Konsol")
        self.resize(1280, 720)
        self.setStyleSheet(self._styles())
        self.showMaximized()


        self.cap = None
        self.kameraIndex = kameraIndex
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._grab_frame)

        self.telemetry = Telemetri()

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)
        
        #TEST MODU İÇİN YORUM SATIRINA ALINMIŞTIR BUNU AÇIN ARDUNİO BAĞLIYKEN
        
        # self.serial_thread = SerialReaderThread('/dev/ttyUSB0', 9600)
        # self.serial_thread.dataReceived.connect(self.serialVeri)
        # self.serial_thread.start()
        
        top = QtWidgets.QHBoxLayout()
        top.setSpacing(12)
        root.addLayout(top, 1)

        # Video alanÄ±
        self.video = VideoWidget()
        video_container = QtWidgets.QFrame()
        video_container.setObjectName("videoContainer")
        vlay = QtWidgets.QVBoxLayout(video_container)
        vlay.setContentsMargins(0, 0, 0, 0)
        vlay.addWidget(self.video)
        top.addWidget(video_container, 2)

        ctrl = QtWidgets.QFrame()
        ctrl.setObjectName("controlPanel")
        ctrl.setMinimumWidth(360)
        top.addWidget(ctrl, 1)
        cl = QtWidgets.QVBoxLayout(ctrl)
        cl.setSpacing(10)

        # Arm/Disarm/Connect
        # Arm/Disarm/Connect
        row1 = QtWidgets.QHBoxLayout()

        self.btnChangeCamera = self._buton("Change Camera", "blue")
        self.btnChangeCamera.clicked.connect(self._kameraDegistir)

        self.btnConfig = self._buton("Config", "gray")
        self.btnConnect = self._buton("Connect", "green")

        self.btnConfig.clicked.connect(lambda: self._configMenu())
        self.btnConnect.clicked.connect(self.toggle_connection)

        row1.addWidget(self.btnChangeCamera)
        row1.addWidget(self.btnConfig)
        row1.addWidget(self.btnConnect)
        cl.addLayout(row1)

        # Eski grid layout yerine joystick gÃ¶rÃ¼nÃ¼mÃ¼
        configLayout = QtWidgets.QHBoxLayout()

        # Sol joystick (normal X-Y hareket)
        self.joystickXY = JoystickWidget()

        # SaÄ joystick (sadece Y ekseni hareketli)
        self.joystickY = JoystickWidget(lock_x=True)

        configLayout.addWidget(self.joystickXY)
        configLayout.addWidget(self.joystickY)
        cl.addLayout(configLayout)
               
        # Piston/Joystick
        row3 = QtWidgets.QHBoxLayout()
        self.lblPiston = QtWidgets.QLabel("Piston Mode: None")
        self.lblJoy = QtWidgets.QLabel("Joystick Active")
        row3.addWidget(self.lblPiston)
        row3.addStretch(1)
        row3.addWidget(self.lblJoy)
        cl.addLayout(row3)

        self.btnMode = self._buton("Mode: Joystick", "green")
        self.btnMode.clicked.connect(lambda: self._log_action("Mode: Joystick"))
        cl.addWidget(self.btnMode)
        cl.addStretch(1)

        #Joystick layout ve alt panel
        configLayout = QtWidgets.QVBoxLayout()

        # Sol ve sağ joystick
        joysticks = QtWidgets.QHBoxLayout()
        self.joystickXY = JoystickWidget()
        self.joystickY = JoystickWidget(lock_x=True)

        configLayout.addLayout(joysticks)

        # X, Y, Z değerlerini gösteren satırlar
        coordLayout = QtWidgets.QHBoxLayout()
        self.lblXVal = QtWidgets.QLabel("X: 0")
        self.lblYVal = QtWidgets.QLabel("Y: 0")
        self.lblZVal = QtWidgets.QLabel("Z: 0")

        for lbl in [self.lblXVal, self.lblYVal, self.lblZVal]:
            lbl.setAlignment(QtCore.Qt.AlignCenter)
            lbl.setStyleSheet("color: #e9e9ea; font-weight: bold;")

        coordLayout.addWidget(self.lblXVal)
        coordLayout.addWidget(self.lblYVal)
        coordLayout.addWidget(self.lblZVal)
        configLayout.addLayout(coordLayout)

        self._load_last_from_db()

        cl.addLayout(configLayout)


        # self.lblImage = QtWidgets.QLabel()
        # self.lblImage.setAlignment(QtCore.Qt.AlignCenter)
        # self.lblImage.setPixmap(QtGui.QPixmap("su.png").scaled(300, 300, QtCore.Qt.KeepAspectRatio))
        # cl.addWidget(self.lblImage)

        # Alt bar: Telemetri + Light
        bottom = QtWidgets.QHBoxLayout()
        bottom.setSpacing(12)
        root.addLayout(bottom, 0)

        # Telemetry
        self.telemetryBox = self._telemetry_panel()
        bottom.addWidget(self.telemetryBox, 3)

        # Butonları dikeyde toplayacak layout
        btnLayout = QtWidgets.QVBoxLayout()

        self.btnLight = self._buton("Turn off lights", "red")
        self.btnLight.setFixedHeight(56)
        btnLayout.addWidget(self.btnLight)

        self.btnOtonom = self._buton("Otonom", "red")
        self.btnOtonom.setFixedHeight(56)
        btnLayout.addWidget(self.btnOtonom)

        self.btnKol = self._buton("Kol Kapat", "red")
        self.btnKol.setFixedHeight(56)
        self.btnKol.clicked.connect(self._kolKontrol)
        btnLayout.addWidget(self.btnKol)

        # Buton grubunu ekle
        bottom.addLayout(btnLayout, 1)

        # Compass
        self.compass = PusulaWidget()
        rightBottom = QtWidgets.QFrame()
        rb = QtWidgets.QVBoxLayout(rightBottom)
        rb.addWidget(self.compass)
        bottom.addWidget(rightBottom, 2)


        # Timer: sahte telemetri guncelle
        self.telemetryTimer = QtCore.QTimer(self)
        self.telemetryTimer.timeout.connect(self._update_fake_telemetry)
        self.telemetryTimer.start(100)

        self.toggle_connection()  # try open camera
        self.serialVeri2(data=None)

    # ------------------------- UI Parcalari -------------------------

    def _load_last_from_db(self):
        import sqlite3, os
        db_path = os.path.join(os.path.dirname(__file__), "db", "config.db")
        if os.path.exists(db_path):
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()
            cursor.execute("SELECT x, y, z FROM otonom_config ORDER BY id DESC LIMIT 1")
            row = cursor.fetchone()
            if row:
                lastX, lastY, lastZ = row
            else:
                lastX = lastY = lastZ = None
            conn.close()
        else:
            lastX = lastY = lastZ = None

        self.lblXVal.setText(f"X: {lastX}")
        self.lblYVal.setText(f"Y: {lastY}")
        self.lblZVal.setText(f"Z: {lastZ}")

    def _buton(self, text: str, kind: str = "dark") -> QtWidgets.QPushButton:
        b = QtWidgets.QPushButton(text)
        b.setProperty("kind", kind)
        b.setMinimumHeight(42)
        return b
        
    def serialVeri(self, data):
        import random
        # Gelen veriyi ay?kla
        values = data.split(',')

        if len(values) < 2:
            return

        x_value = int(values[0])  # 1. de?er x ekseni olarak al
        y_value = int(values[1])  # 2. de?er y ekseni olarak al
        z_value = int(values[2])
        light_value = int(values[3])
        arm_value = int(values[4])
        otonom_value = int(values[5])

        otonom_value = 1
#------------ Light Value Control ------------
        if light_value == 1:
            print("Işık açılıyor")
            self.btnLight.setText("Işık: Aktif")
            self.btnLight.setProperty("kind", "green")

            # Kodlar Buraya...

        elif light_value == 0:
            print("ışık Kapalı")
            self.btnLight.setText("Işık: Kapalı")
            self.btnLight.setProperty("kind", "red")

        else:
            print("Işık Value Error")
            self.btnLight.setText("Işık: Hata")
            self.btnLight.setProperty("kind", "yellow")

        self.style().unpolish(self.btnLight)
        self.style().polish(self.btnLight)

#------------ Arm Value Control ------------

        if arm_value == 1:
            print("kol açılıyor")
            self.btnOtonom.setText("Kol: Aktif")
            self.btnOtonom.setProperty("kind", "green")

            # Kodlar Buraya...

        elif arm_value == 0:
            print("otonom Kapalı")
            self.btnOtonom.setText("Otonom: Kapalı")
            self.btnOtonom.setProperty("kind", "red")

        else:
            print("Arm Value Error")
            self.btnOtonom.setText("Arm: Hata")
            self.btnOtonom.setProperty("kind", "yellow")

        self.style().unpolish(self.btnOtonom)
        self.style().polish(self.btnOtonom)

#------------ Otonom Value Control ------------

        if otonom_value == 1:
            print("otonom açılıyor")
            QtWidgets.QMessageBox.information(self,"Başarılı", "Otonom Göreve Geçildi.")
            self.btnOtonom.setText("Otonom: Aktif")
            self.btnOtonom.setProperty("kind", "green")
            # Kodlar buraya...

        elif otonom_value == 0:
            print("otonom Kapalı")
            self.btnOtonom.setText("Otonom: Kapalı")
            self.btnOtonom.setProperty("kind", "red")

        else:
            print("otonom Value Error")
            self.btnOtonom.setText("Otonom: Hata")
            self.btnOtonom.setProperty("kind", "yellow")

        self.style().unpolish(self.btnOtonom)
        self.style().polish(self.btnOtonom)

        # Joystick pozisyonunu gsncelle
        self.joystickXY.update_position(x_value, y_value)
        self.joystickY.update_position(
            512,                       # X sabit
            random.randint(400, 600)   # Y sabit
        )

    def serialVeri2(self, data=None):
        import random
        if data is None:  # Eğer dışarıdan veri gelmediyse
            x = random.randint(0, 100)
            y = random.randint(0, 100)
            z = random.randint(0, 100)
            light = random.randint(0, 1)
            arm = random.randint(0, 1)
            otonom = 0
            data = f"{x},{y},{z},{light},{arm},{otonom}"
            print(data)

        values = data.split(',')

        if otonom == 1:
            print("otonom açılıyor")
            QtWidgets.QMessageBox.information(self,"Başarılı", "Otonom Göreve Geçildi.")
            self.btnOtonom.setText("Otonom: Aktif")
            self.btnOtonom.setProperty("kind", "green")
            # Otonom kodları buraya...

        elif otonom == 0:
            print("otonom Kapalı")
            self.btnOtonom.setText("Otonom: Kapalı")
            self.btnOtonom.setProperty("kind", "red")

        else:
            print("otonom Value Error")
            self.btnOtonom.setText("Otonom: Hata")
            self.btnOtonom.setProperty("kind", "yellow")

        self.style().unpolish(self.btnOtonom)
        self.style().polish(self.btnOtonom)

        
        if arm == 1:
            print("kol açılıyor")
            self.btnKol.setText("Kol: Aktif")
            self.btnKol.setProperty("kind", "green")

        elif arm == 0:
            print("otonom Kapalı")
            self.btnKol.setText("Otonom: Kapalı")
            self.btnKol.setProperty("kind", "red")

        else:
            print("Arm Value Error")
            self.btnKol.setText("Arm: Hata")
            self.btnKol.setProperty("kind", "yellow")


        self.style().unpolish(self.btnKol)
        self.style().polish(self.btnKol)

        if light == 1:
            print("Işık açılıyor")
            self.btnLight.setText("Işık: Aktif")
            self.btnLight.setProperty("kind", "green")

            # Kodlar Buraya...

        elif light == 0:
            print("ışık Kapalı")
            self.btnLight.setText("Işık: Kapalı")
            self.btnLight.setProperty("kind", "red")

        else:
            print("Işık Value Error")
            self.btnLight.setText("Işık: Hata")
            self.btnLight.setProperty("kind", "yellow")

        self.style().unpolish(self.btnLight)
        self.style().polish(self.btnLight)

    def veriAlmayiDurdur(self):
        self.serial_thread.stop()
        print("Thread durduruldu.")

    def eylemleriDurdur(self, event):
        self.serial_thread.stop()
        event.accept()
    

    def _kameraDegistir(self):
        self.kameraIndex += 1
        # 3 kameradan sonra tekrar 0'a dÃ¶nsÃ¼n (opsiyonel)
        if self.kameraIndex > 2:  # burada 2, sistemdeki max kamera sayÄ±sÄ±na gÃ¶re deÄiÅtirilebilir
            self.kameraIndex = 0

        # Mevcut kamerayÄ± kapat
        if self.cap and self.cap.isOpened():
            self.cap.release()

        # Yeni kamerayÄ± baÅlat
        self.cap = cv2.VideoCapture(self.kameraIndex)
        if not self.cap.isOpened():
            QtWidgets.QMessageBox.warning(self, "Kamera HatasÄ±",
                                        f"Kamera {self.kameraIndex} aÃ§Ä±lamadÄ±!")
        else:
            self._log_action(f"Kamera {self.kameraIndex} seÃ§ildi")


    def _telemetry_panel(self):
        box = QtWidgets.QFrame()
        box.setObjectName("telemetry")
        lay = QtWidgets.QHBoxLayout(box)
        lay.setContentsMargins(16, 16, 16, 16)
        lay.setSpacing(32)

        # Telemetri label'larÄ±
        self.lblYaw = QtWidgets.QLabel()
        self.lblPitch = QtWidgets.QLabel()
        self.lblRoll = QtWidgets.QLabel()
        self.lblX = QtWidgets.QLabel()
        self.lblY = QtWidgets.QLabel()
        self.lblZ = QtWidgets.QLabel()

        items = [
            ("Yaw", self.lblYaw),
            ("Pitch", self.lblPitch),
            ("Roll", self.lblRoll),
            ("X Ekseni", self.lblX),
            ("Y Ekseni", self.lblY),
            ("Z Ekseni", self.lblZ),
        ]

        for key_text, value_lbl in items:
            col = QtWidgets.QVBoxLayout()
            key = QtWidgets.QLabel(key_text)
            key.setObjectName("key")
            key.setAlignment(QtCore.Qt.AlignCenter)
            value_lbl.setAlignment(QtCore.Qt.AlignCenter)
            col.addWidget(key)
            col.addWidget(value_lbl)
            lay.addLayout(col)

        self._refresh_telemetry_labels()
        return box



    def _refresh_telemetry_labels(self):
        t = self.telemetry
        self.lblYaw.setText(f"{t.yaw:0.1f}Â°")
        self.lblPitch.setText(f"{t.pitch:0.1f}Â°")
        self.lblRoll.setText(f"{t.roll:0.1f}Â°")
        self.lblX.setText(f"{t.x_axis:0.2f}")
        self.lblY.setText(f"{t.y_axis:0.2f}")
        self.lblZ.setText(f"{t.z_axis:0.2f}")


    def _styles(self):
        return (
            """
            QWidget { background-color: #0f0f10; color: #e9e9ea; font-family: 'Oswald', 'Segoe UI', sans-serif; }
            QLabel#key { color: #b9b9ba; }
            QFrame#videoContainer { background:#000; border-radius: 10px; }
            QFrame#controlPanel { background:#1a1a1b; border-radius: 10px; padding: 12px; }
            QFrame#telemetry { background:#151516; border-radius: 10px; }
            QPushButton { border: none; border-radius: 8px; padding: 10px 14px; font-weight: 600; }
            QPushButton[kind="dark"] { background: #2b2c2f; color: #e9e9ea; }
            QPushButton[kind="gray"] { background: #3a3b3f; }
            QPushButton[kind="green"] { background: #16a34a; }
            QPushButton[kind="yellow"] { background: #f59e0b; color:#111; }
            QPushButton[kind="red"] { background: #ef4444; }
            QToolButton { background: #2b2c2f; border-radius: 14px; padding:6px 10px; }
            QLineEdit { background:#2b2c2f; border-radius:8px; padding:10px; color:#9fa0a4; }
            """
        )

    # ------------------------- Kamera & Telemetri -------------------------
    def toggle_connection(self):
        if self.cap is None:
            self._open_camera()
        else:
            self._close_camera()



    def _configMenu(self):
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Eksen Konfigürasyonu")

        layout = QtWidgets.QFormLayout(dialog)

        # X, Y, Z input alanları (sadece integer)
        spinX = QtWidgets.QSpinBox()
        spinX.setRange(-9999, 9999)

        spinY = QtWidgets.QSpinBox()
        spinY.setRange(-9999, 9999)

        spinZ = QtWidgets.QSpinBox()
        spinZ.setRange(-9999, 9999)

        layout.addRow("X Eksen:", spinX)
        layout.addRow("Y Eksen:", spinY)
        layout.addRow("Z Eksen:", spinZ)

        # Butonlar (Kaydet / İptal)
        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Save | QtWidgets.QDialogButtonBox.Cancel)
        layout.addRow(btns)

        # db klasörünü kontrol et, yoksa oluştur
        base_dir = os.path.dirname(__file__)
        db_dir = os.path.join(base_dir, "db")
        if not os.path.exists(db_dir):
            os.makedirs(db_dir)

        db_path = os.path.join(db_dir, "config.db")
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()

        # Tablo yoksa oluştur
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS otonom_config (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                x INTEGER,
                y INTEGER,
                z INTEGER,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        """)
        conn.commit()

        def kaydet():
            x = spinX.value()
            y = spinY.value()
            z = spinZ.value()
            # Veritabanına kaydet
            cursor.execute("INSERT INTO otonom_config (x, y, z) VALUES (?, ?, ?)", (x, y, z))
            conn.commit()
            self._load_last_from_db()
            QtWidgets.QMessageBox.information(dialog, "Başarılı", f"X: {x}, Y: {y}, Z: {z} olarak veriler kaydedildi.")
            dialog.accept()

        btns.accepted.connect(kaydet)
        btns.rejected.connect(dialog.reject)

        dialog.exec_()
        conn.close()



    def _open_camera(self):
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # MJPEG formatını zorla
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)  # Çözünürlük ayarı
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)  # Çözünürlük ayarı

        if not self.cap.isOpened():
            self.cap.release()
            self.cap = None
            QtWidgets.QMessageBox.warning(self, "Kamera", "Kamera açılamadı.")
            return
        self.timer.start(33)  # ~30 FPS
        self.btnConnect.setText("Disconnect")

    def _close_camera(self):
        self.timer.stop()
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        self.video.setFrame(np.zeros((360, 640, 3), dtype=np.uint8))
        self.btnConnect.setText("Connect")

    def _grab_frame(self):
        if not self.video.isPlaying():
            return
        if self.cap is None:
            return
        ok, frame = self.cap.read()
        if not ok:
            return
        self.video.setFrame(frame)

    def _update_fake_telemetry(self):
        t = time.time()
        self.telemetry.yaw = (self.telemetry.yaw + 0.8) % 360
        self.telemetry.roll = 15 * np.sin(t * 0.7)
        self.telemetry.pitch = 8 * np.sin(t * 0.9)
        self.telemetry.depth_m = max(0.0, 1.2 * np.sin(t * 0.3) + 1.2)
        self.telemetry.temperature = 26.5 + 0.6 * np.sin(t * 0.2)

        # Rastgele X, Y, Z deÄerleri (-5 ile 5 arasÄ±)
        self.telemetry.x_axis = np.random.uniform(-5, 5)
        self.telemetry.y_axis = np.random.uniform(-5, 5)
        self.telemetry.z_axis = np.random.uniform(-5, 5)

        self._refresh_telemetry_labels()
        self.compass.setYaw(self.telemetry.yaw)


    # ------------------------- Ä°Ålevler -------------------------
    def _kolKontrol(self, on: bool):
        if self.btnKol.text().endswith("Kol Kapat"):
            self.btnKol.setText("Kol Aç")
            self.btnKol.setProperty("kind", "green")
        else:
            self.btnKol.setText("Kol Kapat")
            self.btnKol.setProperty("kind", "red")
        self.style().unpolish(self.btnKol)
        self.style().polish(self.btnKol)
        self._log_action(self.btnKol.text())
        
    # def _isikKontrol(self):
    #     if self.btnLight.text().endswith("Turn off lights"):
    #         self.btnLight.setText("Turn on lights")
    #         self.btnLight.setProperty("kind", "green")
    #     else:
    #         self.btnLight.setText("Turn off lights")
    #         self.btnLight.setProperty("kind", "red")
    #     self.style().unpolish(self.btnLight)
    #     self.style().polish(self.btnLight)
    #     self._log_action(self.btnLight.text())

    def _log_action(self, msg: str):
        print("[ACTION]", msg)

    def closeEvent(self, event: QtGui.QCloseEvent):
        self._close_camera()
        super().closeEvent(event)
        

# ----------------------------- Main -----------------------------
def main():
    app = QtWidgets.QApplication(sys.argv)
    win = ROVKonsol(kameraIndex=1)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
