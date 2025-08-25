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
import RPi.GPIO as GPIO




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

        # Kuzey iÃï¿½areti
        painter.setPen(QtGui.QPen(QtGui.QColor(180, 180, 180), 1))
        painter.setBrush(QtGui.QColor(70, 70, 70))
        painter.drawEllipse(center, 6, 6)

        # ÃÂ°bre (yaw)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(100, 200, 255))
        angle = np.deg2rad(self._yaw - 90)
        x = center.x() + int((radius - 24) * np.cos(angle))
        y = center.y() + int((radius - 24) * np.sin(angle))
        path = QtGui.QPainterPath()
        path.moveTo(center)
        path.lineTo(x, y)
        painter.drawPath(path)

        # Merkez artÃÂ±
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 180), 2))
        painter.drawLine(center.x() - 12, center.y(), center.x() + 12, center.y())
        painter.drawLine(center.x(), center.y() - 12, center.x(), center.y() + 12)

        # YazÃÂ±: derece
        painter.setPen(QtGui.QColor(220, 220, 220))
        painter.setFont(QtGui.QFont("Oswald", 12))
        painter.drawText(self.rect(), QtCore.Qt.AlignBottom | QtCore.Qt.AlignHCenter, f"{self._yaw:.1f}ÃÂ°")
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

        # DÃÂ±Ãï¿½ kare
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
        """Analog veriyi gÃÂ¼ncelle"""
        if not self.lock_x:
            self.pos_x = max(0, min(1024, x))
        if not self.lock_y:
            self.pos_y = max(0, min(1024, y))
        self.update()
        
class SerialReaderThread(QThread):
    # Gelen veriyi GUI'ye iletmek icin sinyal
    dataReceived = pyqtSignal(str)
    def __init__(self, port="/dev/ttyVirtualArduino", baudrate=9600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self._running = True

    def run(self):
        ser = serial.Serial(self.port, self.baudrate, timeout=1)
        time.sleep(2) # Baglantinin oturmasi icin bekle

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

        
        #TEST MODU ICIN YORUM SATIRINA ALINMISTIR BUNU ACIN ARDUNIO BAGLIYKEN

        
        top = QtWidgets.QHBoxLayout()
        top.setSpacing(12)
        root.addLayout(top, 1)

        # Video alanÃÂ±
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

        # Eski grid layout yerine joystick gÃÂ¶rÃÂ¼nÃÂ¼mÃÂ¼
        configLayout = QtWidgets.QHBoxLayout()

        # Sol joystick (normal X-Y hareket)
        self.joystickXY = JoystickWidget()

        # SaÃï¿½ joystick (sadece Y ekseni hareketli)
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

        # Sol ve sag joystick
        joysticks = QtWidgets.QHBoxLayout()
        self.joystickXY = JoystickWidget()
        self.joystickY = JoystickWidget(lock_x=True)

        configLayout.addLayout(joysticks)

        # X, Y, Z degerlerini gosteren satirlar
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

        # Butonlari dikeyde toplayacak layout
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
        values = data.split(',')
        print(values)
        print("ddd: ", data)

        if len(values) < 2:
            return

        x_value = int(values[0])  # 1. de?er x ekseni olarak al
        y_value = int(values[1])  # 2. de?er y ekseni olarak al
        z_value = int(values[2])
        light_value = 0
        arm_value = 0
        otonom_value = 0

        otonom_value = 0
        
        
        print("Burdaym")
        print(x_value)
        print(y_value)
        print(z_value)
        
        
        MOTOR_PINS = {
        "motor1": 26, "motor2": 19, "motor3": 13, "motor4": 6,  # SaÃï¿½/sol motorlar
        "motor5": 21, "motor6": 20, "motor7": 16, "motor8": 5 }
        
        GPIO.setmode(GPIO.BCM)

        for motor in MOTOR_PINS.values():
            GPIO.setup(motor, GPIO.OUT)

        pwm_motorlar = {}
        for motor, pin in MOTOR_PINS.items():
            pwm_motorlar[motor] = GPIO.PWM(pin, 50)  # 50Hz
            pwm_motorlar[motor].start(0)
            
            
        def set_motor_hizi(motor_adi, pulse):
            duty = pulse / 20000 * 100  
            duty = max(0, min(duty, 100))  # Duty %0 ile %100 arasÃÂ±nda 
            pwm_motorlar[motor_adi].ChangeDutyCycle(duty)
            print(f"{motor_adi} hizi: {pulse} us (duty: {duty:.2f}%)")

        def kalibrasyon():
            print("Motor kalibrasyonu baslatiliyor...")
            # Her motoru kalibre et
            for motor in MOTOR_PINS.keys():
                set_motor_hizi(motor, 1500)  # Motoru
            time.sleep(2)
            print("Kalibrasyon tamamlandi")

        def map_func(deger, giris_min, giris_max, cikis_min, cikis_max):
            return (deger - giris_min) * (cikis_max - cikis_min) / (giris_max - giris_min) + cikis_min
            
        
            
       
        



#----- Light Value Control ------------
        if light_value == 1:
            print("Isik aciliyor")
            self.btnLight.setText("Isik: Aktif")
            self.btnLight.setProperty("kind", "green")

            # Kodlar Buraya...

        elif light_value == 0:
            print("isik Kapali")
            self.btnLight.setText("Isik: Kapali")
            self.btnLight.setProperty("kind", "red")

        else:
            print("Isik Value Error")
            self.btnLight.setText("Isik: Hata")
            self.btnLight.setProperty("kind", "yellow")

        self.style().unpolish(self.btnLight)
        self.style().polish(self.btnLight)

#------------ Arm Value Control ------------

        if arm_value == 1:
            print("kol aciliyor")
            self.btnOtonom.setText("Kol: Aktif")
            self.btnOtonom.setProperty("kind", "green")

            # Kodlar Buraya...

        elif arm_value == 0:
            print("otonom Kapali")
            self.btnOtonom.setText("Otonom: Kapali")
            self.btnOtonom.setProperty("kind", "red")

        else:
            print("Arm Value Error")
            self.btnOtonom.setText("Arm: Hata")
            self.btnOtonom.setProperty("kind", "yellow")

        self.style().unpolish(self.btnOtonom)
        self.style().polish(self.btnOtonom)

#------------ Otonom Value Control ------------

        if otonom_value == 1:
            print("otonom aciliyor")
            QtWidgets.QMessageBox.information(self,"Basarili", "Otonom Goreve Gecildi.")
            self.btnOtonom.setText("Otonom: Aktif")
            self.btnOtonom.setProperty("kind", "green")
            # Kodlar buraya...

        elif otonom_value == 0:
            print("otonom Kapali")
            self.btnOtonom.setText("Otonom: Kapali")
            self.btnOtonom.setProperty("kind", "red")

        else:
            print("otonom Value Error")
            self.btnOtonom.setText("Otonom: Hata")
            self.btnOtonom.setProperty("kind", "yellow")

        self.style().unpolish(self.btnOtonom)
        self.style().polish(self.btnOtonom)
        #############################################3



            # Joystick pozisyonunu gsncelle
        self.joystickXY.update_position(x_value, y_value)
        self.joystickY.update_position(
            512,                       # X sabit
            random.randint(400, 600)   # Y sabit
        )

    def serialVeri2(self, data=None):
        import random
        if data is None:  # Eger disaridan veri gelmediyse
            x = random.randint(0, 100)
            y = random.randint(0, 100)
            z = random.randint(0, 100)
            light = 0
            arm = 0
            otonom = 0
            data = f"{x},{y},{z},{light},{arm},{otonom}"
            print(data)

        values = data.split(',')

        if otonom == 1:
            print("otonom aciliyor")
            QtWidgets.QMessageBox.information(self,"Basarili", "Otonom Goreve Gecildi.")
            self.btnOtonom.setText("Otonom: Aktif")
            self.btnOtonom.setProperty("kind", "green")
            # Otonom kodlari buraya...

        elif otonom == 0:
            print("otonom Kapali")
            self.btnOtonom.setText("Otonom: Kapali")
            self.btnOtonom.setProperty("kind", "red")

        else:
            print("otonom Value Error")
            self.btnOtonom.setText("Otonom: Hata")
            self.btnOtonom.setProperty("kind", "yellow")

        self.style().unpolish(self.btnOtonom)
        self.style().polish(self.btnOtonom)

        
        if arm == 1:
            print("kol aciliyor")
            self.btnKol.setText("Kol: Aktif")
            self.btnKol.setProperty("kind", "green")

        elif arm == 0:
            print("otonom Kapali")
            self.btnKol.setText("Otonom: Kapali")
            self.btnKol.setProperty("kind", "red")

        else:
            print("Arm Value Error")
            self.btnKol.setText("Arm: Hata")
            self.btnKol.setProperty("kind", "yellow")


        self.style().unpolish(self.btnKol)
        self.style().polish(self.btnKol)

        if light == 1:
            print("Isik aciliyor")
            self.btnLight.setText("Isik: Aktif")
            self.btnLight.setProperty("kind", "green")

            # Kodlar Buraya...

        elif light == 0:
            print("isik Kapali")
            self.btnLight.setText("Isik: Kapali")
            self.btnLight.setProperty("kind", "red")

        else:
            print("Isik Value Error")
            self.btnLight.setText("Isik: Hata")
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
        # 3 kameradan sonra tekrar 0'a dÃÂ¶nsÃÂ¼n (opsiyonel)
        if self.kameraIndex > 2:  # burada 2, sistemdeki max kamera sayÃÂ±sÃÂ±na gÃÂ¶re deÃï¿½iÃï¿½tirilebilir
            self.kameraIndex = 0

        # Mevcut kamerayÃÂ± kapat
        if self.cap and self.cap.isOpened():
            self.cap.release()

        # Yeni kamerayÃÂ± baÃï¿½lat
        self.cap = cv2.VideoCapture(self.kameraIndex)
        if not self.cap.isOpened():
            QtWidgets.QMessageBox.warning(self, "Kamera HatasÃÂ±",
                                        f"Kamera {self.kameraIndex} aÃÂ§ÃÂ±lamadÃÂ±!")
        else:
            self._log_action(f"Kamera {self.kameraIndex} seÃÂ§ildi")

    def _telemetry_panel(self):
        box = QtWidgets.QFrame()
        box.setObjectName("telemetry")
        lay = QtWidgets.QHBoxLayout(box)
        lay.setContentsMargins(16, 16, 16, 16)
        lay.setSpacing(32)

        # Telemetri label'larÃÂ±
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
        self.lblYaw.setText(f"{t.yaw:0.1f}ÃÂ°")
        self.lblPitch.setText(f"{t.pitch:0.1f}ÃÂ°")
        self.lblRoll.setText(f"{t.roll:0.1f}ÃÂ°")
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
        dialog.setWindowTitle("Eksen Konfigurasyonu")

        layout = QtWidgets.QFormLayout(dialog)

        # X, Y, Z input alanlari (sadece integer)
        spinX = QtWidgets.QSpinBox()
        spinX.setRange(-9999, 9999)

        spinY = QtWidgets.QSpinBox()
        spinY.setRange(-9999, 9999)

        spinZ = QtWidgets.QSpinBox()
        spinZ.setRange(-9999, 9999)

        layout.addRow("X Eksen:", spinX)
        layout.addRow("Y Eksen:", spinY)
        layout.addRow("Z Eksen:", spinZ)

        # Butonlar (Kaydet / Iptal)
        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Save | QtWidgets.QDialogButtonBox.Cancel)
        layout.addRow(btns)

        # db klasorunu kontrol et, yoksa olustur
        base_dir = os.path.dirname(__file__)
        db_dir = os.path.join(base_dir, "db")
        if not os.path.exists(db_dir):
            os.makedirs(db_dir)

        db_path = os.path.join(db_dir, "config.db")
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()

        # Tablo yoksa olustur
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
            # Veritabanina kaydet
            cursor.execute("INSERT INTO otonom_config (x, y, z) VALUES (?, ?, ?)", (x, y, z))
            conn.commit()
            self._load_last_from_db()
            QtWidgets.QMessageBox.information(dialog, "Basarili", f"X: {x}, Y: {y}, Z: {z} olarak veriler kaydedildi.")
            dialog.accept()

        btns.accepted.connect(kaydet)
        btns.rejected.connect(dialog.reject)

        dialog.exec_()
        conn.close()

    def _open_camera(self):
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # MJPEG formatini zorla
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)  # Cozunurluk ayari
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)  # Cozunurluk ayari

        if not self.cap.isOpened():
            self.cap.release()
            self.cap = None
            QtWidgets.QMessageBox.warning(self, "Kamera", "Kamera acilamadi.")
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

        # Rastgele X, Y, Z deÃï¿½erleri (-5 ile 5 arasÃÂ±)
        self.telemetry.x_axis = np.random.uniform(-5, 5)
        self.telemetry.y_axis = np.random.uniform(-5, 5)
        self.telemetry.z_axis = np.random.uniform(-5, 5)

        self._refresh_telemetry_labels()
        self.compass.setYaw(self.telemetry.yaw)


    # ------------------------- ÃÂ°Ãï¿½levler -------------------------
    def _kolKontrol(self, on: bool):
        if self.btnKol.text().endswith("Kol Kapat"):
            self.btnKol.setText("Kol Ac")
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

def motor():

    try:
        ser = serial.Serial('/dev/ttyVirtualArduino', 9600, timeout=0.1)  # Non-blocking
        time.sleep(0.1)
        
    except serial.SerialException:
        print("Seri port bulunamadÃÂ± /dev/ttyUSB0")
        ser = None

    # Motor pinleri
    MOTOR_PINS = {
        "motor1": 26, "motor2": 19, "motor3": 13, "motor4": 6,  
        "motor5": 21, "motor6": 20, "motor7": 16, "motor8": 5    
    }

    GPIO.setmode(GPIO.BCM)
    

    # Motor pinlerini giri/ÃÂ§ÃÂ±kÃÂ± olarak ayarla
    for motor in MOTOR_PINS.values():
        GPIO.setup(motor, GPIO.OUT)

    # PWM baÃt
    pwm_motorlar = {}
    for motor, pin in MOTOR_PINS.items():
        pwm_motorlar[motor] = GPIO.PWM(pin, 50)  # 50Hz
        pwm_motorlar[motor].start(0)
        
    for motor, pin in MOTOR_PINS.items():
        if motor not in pwm_motorlar:
            pwm_motorlar[motor] = GPIO.PWM(pin, 50)  # 50Hz
            pwm_motorlar[motor].start(0)

    # Motor hÃÂ±zÃÂ±nÃÂ± ayarlamak iÃÂ§in fonksiyon
    def set_motor_hizi(motor_adi, pulse):
        duty = pulse / 20000 * 100  
        duty = max(0, min(duty, 100))  # Duty %0 ile %100 arasÃÂ±nda kalmalÃÂ±
        pwm_motorlar[motor_adi].ChangeDutyCycle(duty)
        print(f"{motor_adi} hÃÂ±zÃÂ±: {pulse} us (duty: {duty:.2f}%)")

    # Kalibrasyon fonksiyonu
    def kalibrasyon():
        print("Motor kalibrasyonu baÃatÃÂ±lÃÂ±yor...")
        # Her motoru kalibre et
        for motor in MOTOR_PINS.keys():
            set_motor_hizi(motor, 1500)  # Motoru hÃÂ±zlÃÂ± balat
        time.sleep(2)
        print("Kalibrasyon tamamlandÃÂ±")
        
    def trim_motor_hizi(motor_adi, pwm_motorlar, increment=5):

        current_duty = pwm_motorlar[motor_adi].duty_cycle  # Mevcut duty cycle'? al
        new_duty = current_duty + increment  # Duty cycle'a trim uygula
        new_duty = max(0, min(new_duty, 100))  # Duty cycle'? %0 ile %100 aras?nda tut

        pwm_motorlar[motor_adi].ChangeDutyCycle(new_duty)  # Yeni de?eri uygula
        print(f"{motor_adi} yeni h?z: {new_duty}%")


    # Veriyi belirli bir aralÃÂ±a haritalama fonksiyonu
    def map_func(deger, giris_min, giris_max, cikis_min, cikis_max):
        return (deger - giris_min) * (cikis_max - cikis_min) / (giris_max - giris_min) + cikis_min

    # Seri porttan veri okuma fonksiyonu
    def oku_seri():
        if ser and ser.in_waiting > 0:
            try:
                data = ser.readline().decode('utf-8').strip()
                su = data.split(",")  # Gelen veriyi virgÃÂ¼lle ayÃÂ±r
                
                print(data)
                return [int(x) for x in su]  # Verileri liste olarak dÃÂ¶ndÃÂ¼r
            except Exception as e:
                print(f"Hata oluÃtu: {e}")
                return None
        return None

    # Ana program
    try:
        kalibrasyon()  # Kalibrasyonu baÃlat
        first_value = None 
        list = []
        list2 = []
        list3 = []
        flag = False
        stop_flag = False

        while True:
            sensor_degerleri = oku_seri()  # Seri porttan veri oku
            if sensor_degerleri is not None:
                xValue = sensor_degerleri[0]  # Gelen verinin ilk elemanÃÂ±nÃÂ± al
                yValue = sensor_degerleri[1]  # SaÃ/sol motor iÃÂ§in gelen veri
                zValue = sensor_degerleri[2]  # YukarÃÂ±/aÃÃÂ± motor iÃÂ§in gelen veri
                list.append(xValue)  # Gelen veriyi listeye ekle
                print(f"Gelen veri: {xValue}")

                if first_value is None:
                    first_value = xValue
                    print(f"ÃÂ°lk veri: {first_value}")
                
                if xValue != first_value and not flag:
                    print(f"ÃÂ°lk farklÃÂ± veri geldi: {xValue}")
                    flag = True

                if flag and xValue == 512:
                    print("512 geldi, motorlar durduruluyor...")
                    stop_flag = True  # MotorlarÃÂ± durdurmak iÃÂ§in flag'ÃÂ± True yap

                if stop_flag:
                    for motor in pwm_motorlar.values():
                        motor.ChangeDutyCycle(1)  # MotorlarÃÂ± durdur 
                    print("Motorlar durdu, ama kaydedilen verilerle motorlar hareket etmeye devam edecek.")
                
                if xValue != 512:  # EÃer gelen veri 512 deÃilse, motorlarÃÂ± tekrar ÃÂ§alÃÂ±ÃtÃÂ±r
                    stop_flag = False  # MotorlarÃÂ± tekrar hareket ettir
            
                else:
                    time.sleep(0.02)
                    

                # Veriyi haritala (0 ile 1024 arasÃÂ±nda)
                yPulse = int(map_func(yValue, 0, 1024, 1000, 2000))
                yPulseTers = int(map_func(yValue, 1024, 0, 1000, 2000))
                
                zPulse = int(map_func(zValue, 0, 1024, 1000, 2000))
                zPulseTers = int(map_func(zValue, 1024, 0, 1000, 2000))

                for idx, motor in enumerate(MOTOR_PINS.keys()):
                    if idx < 4:  
                        if motor == "motor4":
                            set_motor_hizi(motor, yPulseTers)
                        else:
                            set_motor_hizi(motor, yPulse)
                            
                    else:
                        if motor == "motor5":
                            set_motor_hizi(motor, zPulseTers)
                            
                        elif motor == "motor6":
                            set_motor_hizi(motor, zPulseTers)
                            
                        else:
                            set_motor_hizi(motor, zPulse)
                            
                    if not stop_flag and idx < 4:
                        xPulse = int(map_func(xValue, 0, 1024, 1000, 2000))
                        xPulseTers = int(map_func(xValue, 1024, 0, 1000, 2000))  # Veriyi haritala (0 ile 1024 arasÃÂ±nda)
                        
                        if 512 < xValue <= 1024:
                            print(list)
                            for motor in MOTOR_PINS.keys():
                                if motor == "motor1":
                                    set_motor_hizi(motor, xPulseTers)
                                else:
                                    set_motor_hizi(motor, xPulse)

                        elif 0 <= xValue < 512:
                            for motor in MOTOR_PINS.keys():
                                if motor == "motor1":
                                    set_motor_hizi(motor, xPulseTers)
                                else:
                                    set_motor_hizi(motor, xPulse)

                
            time.sleep(0.02)  # 20ms, daha hÃÂ±zlÃÂ± veri almak iÃÂ§in

    except KeyboardInterrupt:
        print("Program durduruldu")
    finally:
        if ser and ser.is_open:
            ser.close()  # Seri portu kapat
        # MotorlarÃÂ± durdur
        for motor in pwm_motorlar.values():
            motor.ChangeDutyCycle(0)  # Motoru durdur
            motor.stop()  # PWM'yi sonlandÃÂ±r
        GPIO.cleanup()  # GPIO temizliÃi yap
        

def main():
    import threading
    app = QtWidgets.QApplication(sys.argv)
    win = ROVKonsol(kameraIndex=1)
    win.show()
    motor_threading = threading.Thread(target=motor)
    motor_threading.daemon = True
    motor_threading.start()
    sys.exit(app.exec_())

if __name__ == "__main__":
        main()

    
