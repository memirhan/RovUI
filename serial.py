import serial
import time

# Seri bağlantı ayarları
ser = serial.Serial('COM8', 9600, timeout=1)
time.sleep(2)  # Bağlantının oturması için bekleme süresi

try:
    while True:
        if ser.in_waiting > 0:  # Gelen veri varsa
            data = ser.readline().decode('utf-8').strip()  # Veriyi okuma ve decode etme
            print("Gelen veri:", data)  # İstersen ekrana yazdır

            su = data.split(",")  # Virgülle ayır

          
            sensor_1 = int(su[0])
            sensor_2 = int(su[1])
            sensor_3 = int(su[2])
            button_1 = int(su[3])
            button_2 = int(su[4])
            button_3 = int(su[5])
            button_4 = int(su[6])

            # Test için ekrana yazdır
            print(sensor_1, sensor_2, sensor_3, button_1, button_2, button_3, button_4)

except KeyboardInterrupt:
    print("Bağlantı sonlandırılıyor...")
finally:
    ser.close()  # Seri bağlantıyı kapatma
