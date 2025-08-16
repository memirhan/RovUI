import RPi.GPIO as GPIO
import time
import serial

# Seri porttan veri okuma
def gelecek_veri():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                su = data.split(",")
                sensor_1 = int(su[0])
                return sensor_1
    except serial.SerialException:
        print("Seri port bulunamadi /dev/ttyUSB0")
        return None
    except KeyboardInterrupt:
        print("Okuma durduruldu")
        return None
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

# Motor pinleri
MOTOR_PINS = {"motor8": 17}
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PINS["motor8"], GPIO.OUT)

# PWM balat
pwm_motor8 = GPIO.PWM(MOTOR_PINS["motor8"], 50)  # 50Hz
pwm_motor8.start(0)

# Motor h ayarlama
def set_motor_hizi(motor_adi, pulse):
    duty = pulse / 20000 * 100
    duty = max(0, min(duty, 100))
    pwm_motor8.ChangeDutyCycle(duty)
    print(f"{motor_adi} hizi: {pulse} us (duty: {duty:.2f}%)")

# Kalibrasyon
def kalibrasyon():
    print("Motor kalibrasyonu baslatiyor...")
    set_motor_hizi("motor8", 2000)
    time.sleep(2)
    set_motor_hizi("motor8", 1000)
    time.sleep(2)
    print("Kalibrasyon tamamlandi")

# Map fonksiyonu
def map_yusuf(deger, giris_min, giris_max, cikis_min, cikis_max):
    return (deger - giris_min) * (cikis_max - cikis_min) / (giris_max - giris_min) + cikis_min
    


# Ana program
try:
    onceki_pulse = None
    while True:
        sensor_degeri = gelecek_veri()
        if sensor_degeri is not None:
            pulse = map_yusuf(sensor_degeri, 0, 1024, 1000, 2000)
            # PWM sadece belli bir ezik itiyse gncellensin
            if onceki_pulse is None or abs(pulse - onceki_pulse) > 5:
                set_motor_hizi("motor8", pulse)
                onceki_pulse = pulse
        time.sleep(0.05)  # PWM gcelleme aral50ms
        
        
except KeyboardInterrupt:
    print("Program durduruldu")
finally:
    pwm_motor8.ChangeDutyCycle(0)
    pwm_motor8.stop()
    GPIO.cleanup()
