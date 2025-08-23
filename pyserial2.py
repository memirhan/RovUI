import serial
import time

# Arduino'nun bağlı olduğu portu gir (Windows: "COM3", Linux/Mac: "/dev/ttyUSB0" veya "/dev/ttyACM0")
ser = serial.Serial("/dev/ttyACM0", 9600)

time.sleep(2)  # Arduino'nun resetten çıkmasını bekle

# Örneğin roliCam komutları göndermek istersen
def set_roli_cam(angle, dim):
    # Arduino'daki SerialTransfer kütüphanesi binary format bekliyor
    # Burada aynı paketi Python’da encode etmen gerek
    # Örnek olarak basit bir string gönderelim (Arduino kodunu da buna göre değiştirmek lazım)
    data = f"{angle},{dim}\n"
    ser.write(data.encode("utf-8"))

# test
set_roli_cam(90, 50)
time.sleep(1)
set_roli_cam(0, 0)