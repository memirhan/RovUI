#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROV Konsol Prototipi
- PyQt5 arayüz
- OpenCV ile kamera akışı (varsayılan kamera index=0)
- Görseldeki konsola benzer düzen
- Pusula (Yaw) göstergesi QPainter ile

Gereksinimler:
    pip install pyqt5 opencv-python numpy

Çalıştırma:
    python rov_konsol.py
"""
import sys
import time
from dataclasses import dataclass

import cv2
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets


# ----------------------------- Yardımcı Bileşenler -----------------------------
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
        # BGR -> RGB
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
        # Overlay: REC ikonu sol üst, play/pause sağ üst
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        # REC simgesi
        pad = 10
        rec_rect = QtCore.QRect(pad, pad, 60, 24)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(0, 0, 0, 120))
        painter.drawRoundedRect(rec_rect, 6, 6)
        painter.setBrush(QtGui.QColor(230, 50, 50))
        painter.drawEllipse(rec_rect.left() + 6, rec_rect.top() + 6, 12, 12)
        painter.setPen(QtGui.QColor(255, 255, 255))
        painter.setFont(QtGui.QFont("Oswald", 12, QtGui.QFont.Bold))
        painter.drawText(rec_rect.adjusted(24, 0, 0, 0), QtCore.Qt.AlignVCenter, "REC")

        # Play/Pause butonları çizim (gösterim; gerçek butonlar üstte)
        # Sağ üst köşede daire arka plan
        r = 22
        x = self.width() - pad - r * 2 - 6
        y = pad
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(0, 0, 0, 120))
        painter.drawEllipse(x, y, r * 2, r * 2)
        painter.drawEllipse(x + r * 2 + 6, y, r * 2, r * 2)

        painter.end()


class CompassWidget(QtWidgets.QWidget):
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

        # Kuzey işareti
        painter.setPen(QtGui.QPen(QtGui.QColor(180, 180, 180), 1))
        painter.setBrush(QtGui.QColor(70, 70, 70))
        painter.drawEllipse(center, 6, 6)

        # İbre (yaw)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(100, 200, 255))
        angle = np.deg2rad(self._yaw - 90)
        x = center.x() + int((radius - 24) * np.cos(angle))
        y = center.y() + int((radius - 24) * np.sin(angle))
        path = QtGui.QPainterPath()
        path.moveTo(center)
        path.lineTo(x, y)
        painter.drawPath(path)

        # Merkez artı
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 180), 2))
        painter.drawLine(center.x() - 12, center.y(), center.x() + 12, center.y())
        painter.drawLine(center.x(), center.y() - 12, center.x(), center.y() + 12)

        # Yazı: derece
        painter.setPen(QtGui.QColor(220, 220, 220))
        painter.setFont(QtGui.QFont("Oswald", 12))
        painter.drawText(self.rect(), QtCore.Qt.AlignBottom | QtCore.Qt.AlignHCenter, f"{self._yaw:.1f}°")
        painter.end()


@dataclass
class Telemetry:
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

        # Dış kare
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
        """Analog veriyi güncelle"""
        if not self.lock_x:
            self.pos_x = max(0, min(1024, x))
        if not self.lock_y:
            self.pos_y = max(0, min(1024, y))
        self.update()



# ----------------------------- Ana Pencere -----------------------------
class ROVConsole(QtWidgets.QMainWindow):
    def __init__(self, camera_index=0):
        super().__init__()
        self.setWindowTitle("ROV Konsol – Prototip")
        self.resize(1280, 720)
        self.setStyleSheet(self._styles())

        self.cap = None
        self.camera_index = camera_index
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._grab_frame)

        self.telemetry = Telemetry()

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        # Üst: Video + Kontrol paneli
        top = QtWidgets.QHBoxLayout()
        top.setSpacing(12)
        root.addLayout(top, 1)

        # Video alanı
        self.video = VideoWidget()
        video_container = QtWidgets.QFrame()
        video_container.setObjectName("videoContainer")
        vlay = QtWidgets.QVBoxLayout(video_container)
        vlay.setContentsMargins(0, 0, 0, 0)
        vlay.addWidget(self.video)

        # Play/Pause gerçek butonları
        pp = QtWidgets.QHBoxLayout()
        pp.setAlignment(QtCore.Qt.AlignRight)
        self.btnPlay = QtWidgets.QToolButton(text="▶")
        self.btnPause = QtWidgets.QToolButton(text="⏸")
        self.btnPlay.clicked.connect(lambda: self.video.setPlaying(True))
        self.btnPause.clicked.connect(lambda: self.video.setPlaying(False))
        pp.addWidget(self.btnPlay)
        pp.addWidget(self.btnPause)
        vlay.addLayout(pp)

        top.addWidget(video_container, 2)

        # Sağ kontrol paneli
        ctrl = QtWidgets.QFrame()
        ctrl.setObjectName("controlPanel")
        ctrl.setMinimumWidth(360)
        top.addWidget(ctrl, 1)
        cl = QtWidgets.QVBoxLayout(ctrl)
        cl.setSpacing(10)

        # Arm/Disarm/Connect
       # Arm/Disarm/Connect
        row1 = QtWidgets.QHBoxLayout()

        # "Motor Aktif" yerine "Kamerayı Değiştir" butonu
        self.btnChangeCamera = self._btn("Değiştir", "blue")
        self.btnChangeCamera.clicked.connect(self._change_camera)

        self.btnDisarm = self._btn("Düzenlenecek", "gray")
        self.btnConnect = self._btn("Connect", "green")

        self.btnDisarm.clicked.connect(lambda: self._set_arm(False))
        self.btnConnect.clicked.connect(self.toggle_connection)

        row1.addWidget(self.btnChangeCamera)
        row1.addWidget(self.btnDisarm)
        row1.addWidget(self.btnConnect)
        cl.addLayout(row1)

        # Eski grid layout yerine joystick görünümü
        joyLayout = QtWidgets.QHBoxLayout()

# Sol joystick (normal X-Y hareket)
        self.joystickXY = JoystickWidget()

        # Sağ joystick (sadece Y ekseni hareketli)
        self.joystickY = JoystickWidget(lock_x=True)

        joyLayout.addWidget(self.joystickXY)
        joyLayout.addWidget(self.joystickY)
        cl.addLayout(joyLayout)

        # Test amaçlı rastgele hareket (gerçek veride kaldır)
        def test_move():
            import random
            self.joystickXY.update_position(
                random.randint(400, 600),  # X
                random.randint(400, 600)   # Y
            )
            self.joystickY.update_position(
                512,                       # X sabit
                random.randint(400, 600)   # Y
            )

        self.joystickTimer = QtCore.QTimer(self)
        self.joystickTimer.timeout.connect(test_move)
        self.joystickTimer.start(500)




        # Piston/Joystick
        row3 = QtWidgets.QHBoxLayout()
        self.lblPiston = QtWidgets.QLabel("Piston Mode: None")
        self.lblJoy = QtWidgets.QLabel("Joystick Active")
        row3.addWidget(self.lblPiston)
        row3.addStretch(1)
        row3.addWidget(self.lblJoy)
        cl.addLayout(row3)

        self.btnMode = self._btn("Mode: Joystick", "green")
        self.btnMode.clicked.connect(lambda: self._log_action("Mode: Joystick"))
        cl.addWidget(self.btnMode)
        cl.addStretch(1)

        # # Fotoğraf Gösterme
        # self.lblImage = QtWidgets.QLabel()
        # self.lblImage.setAlignment(QtCore.Qt.AlignCenter)
        # self.lblImage.setPixmap(QtGui.QPixmap("su.png").scaled(300, 300, QtCore.Qt.KeepAspectRatio))
        # cl.addWidget(self.lblImage)

        # Alt bar: Telemetri + Light
        bottom = QtWidgets.QHBoxLayout()
        bottom.setSpacing(12)
        root.addLayout(bottom, 0)

        self.telemetryBox = self._telemetry_panel()
        bottom.addWidget(self.telemetryBox, 3)

        self.btnLight = self._btn("Işıkları Kapat", "red")
        self.btnLight.setFixedHeight(56)
        self.btnLight.clicked.connect(self._toggle_light)
        bottom.addWidget(self.btnLight, 1)

        # Sağ altta Pusula
        self.compass = CompassWidget()
        rightBottom = QtWidgets.QFrame()
        rb = QtWidgets.QVBoxLayout(rightBottom)
        rb.addWidget(self.compass)
        bottom.addWidget(rightBottom, 2)

        # Timer: sahte telemetri güncelle
        self.telemetryTimer = QtCore.QTimer(self)
        self.telemetryTimer.timeout.connect(self._update_fake_telemetry)
        self.telemetryTimer.start(100)

        # Başlangıçta bağlanmayı dene
        self.toggle_connection()  # try open camera

    # ------------------------- UI Parçaları -------------------------
    def _btn(self, text: str, kind: str = "dark") -> QtWidgets.QPushButton:
        b = QtWidgets.QPushButton(text)
        b.setProperty("kind", kind)
        b.setMinimumHeight(42)
        return b
    

    def _change_camera(self):
        self.camera_index += 1
        # 3 kameradan sonra tekrar 0'a dönsün (opsiyonel)
        if self.camera_index > 2:  # burada 2, sistemdeki max kamera sayısına göre değiştirilebilir
            self.camera_index = 0

        # Mevcut kamerayı kapat
        if self.cap and self.cap.isOpened():
            self.cap.release()

        # Yeni kamerayı başlat
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            QtWidgets.QMessageBox.warning(self, "Kamera Hatası",
                                        f"Kamera {self.camera_index} açılamadı!")
        else:
            self._log_action(f"Kamera {self.camera_index} seçildi")


    def _disabled_box(self, placeholder: str):
        w = QtWidgets.QLineEdit()
        w.setPlaceholderText(placeholder)
        w.setEnabled(False)
        return w

    def _telemetry_panel(self):
        box = QtWidgets.QFrame()
        box.setObjectName("telemetry")
        lay = QtWidgets.QHBoxLayout(box)
        lay.setContentsMargins(16, 16, 16, 16)
        lay.setSpacing(32)

        # Telemetri label'ları
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
        self.lblYaw.setText(f"{t.yaw:0.1f}°")
        self.lblPitch.setText(f"{t.pitch:0.1f}°")
        self.lblRoll.setText(f"{t.roll:0.1f}°")
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

    def _open_camera(self):
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.cap.release()
            self.cap = None
            QtWidgets.QMessageBox.warning(self, "Kamera", "Kamera açılamadı (index 0).")
            return
        self.timer.start(33)  # ~30 FPS
        self.btnConnect.setText("Bağlantıyı Kes")

    def _close_camera(self):
        self.timer.stop()
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        self.video.setFrame(np.zeros((360, 640, 3), dtype=np.uint8))
        self.btnConnect.setText("Bağlan")

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

        # Rastgele X, Y, Z değerleri (-5 ile 5 arası)
        self.telemetry.x_axis = np.random.uniform(-5, 5)
        self.telemetry.y_axis = np.random.uniform(-5, 5)
        self.telemetry.z_axis = np.random.uniform(-5, 5)

        self._refresh_telemetry_labels()
        self.compass.setYaw(self.telemetry.yaw)


    # ------------------------- İşlevler -------------------------
    def _set_arm(self, on: bool):
        self.telemetry.arm = on
        self._refresh_telemetry_labels()
        self._log_action("ARM ON" if on else "ARM OFF")

    def _toggle_light(self):
        if self.btnLight.text().endswith("Kapat"):
            self.btnLight.setText("Işıkları Aç")
            self.btnLight.setProperty("kind", "green")
        else:
            self.btnLight.setText("Işıkları Kapat")
            self.btnLight.setProperty("kind", "red")
        self.style().unpolish(self.btnLight)
        self.style().polish(self.btnLight)
        self._log_action(self.btnLight.text())

    def _log_action(self, msg: str):
        print("[ACTION]", msg)

    def closeEvent(self, event: QtGui.QCloseEvent):
        self._close_camera()
        super().closeEvent(event)


# ----------------------------- Main -----------------------------
def main():
    app = QtWidgets.QApplication(sys.argv)
    win = ROVConsole(camera_index=0)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
