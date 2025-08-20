import RPi.GPIO as GPIO
import time
import serial

# Seri portu ba?la,

#2 ile 4 1 ile de 3 aynÄ± yne dnonb
try:
    ser = serial.Serial('/dev/ttyVirtualArduino', 9600, timeout=0.1)  # Non-blocking
    time.sleep(0.1)
except serial.SerialException:
    print("Seri port bulunamad? /dev/ttyUSB0")
    ser = None

# Motor pinleri
MOTOR_PINS = { "motor1": 26, "motor2": 19, "motor3": 13, "motor4": 6 }

GPIO.setmode(GPIO.BCM)

# Motor pinle?k?? olarak ayarla
for motor in MOTOR_PINS.values():
    GPIO.setup(motor, GPIO.OUT)

# PWM ba?lat
pwm_motorlar = {}
for motor, pin in MOTOR_PINS.items():
    pwm_motorlar[motor] = GPIO.PWM(pin, 50)  # 50Hz
    pwm_motorlar[motor].start(0)

def set_motor_hizi(motor_adi, pulse):
    duty = pulse / 20000 * 100  
    duty = max(0, min(duty, 100))  # Duty %0 ile %100 aras?nda kalmal?

  
    # PWM duty cycle'?n? motor iin uygula
    pwm_motorlar[motor_adi].ChangeDutyCycle(duty)
    print(f"{motor_adi} h?z?: {pulse} us (duty: {duty:.2f}%)")


def kalibrasyon():
    print("Motor kalibrasyonu ba?lat?l?yor...")
    # Her motoru kalibre et
    for motor in MOTOR_PINS.keys():
        set_motor_hizi(motor, 1500)  # Motoru h?zl? al??t?r
    time.sleep(2)

    print("Kalibrasyon tamamland?")

def map_func(deger, giris_min, giris_max, cikis_min, cikis_max):
    # Gelen veriyi belirli bir aral??a haritalama
    return (deger - giris_min) * (cikis_max - cikis_min) / (giris_max - giris_min) + cikis_min

def oku_seri():
    # Seri porttan veri oku
    if ser and ser.in_waiting > 0:
        try:
            data = ser.readline().decode('utf-8').strip()
            su = data.split(",")  # Gelen veriyi virglle ay?r
            return [int(x) for x in su]  # Verileri liste olarak dnr
        except Exception as e:
            print(f"Hata olu?tu: {e}")
            return None
    return None

# Ana program
try:
    kalibrasyon()  # Kalibrasyonu ba?lat
    while True:
        sensor_degerleri = oku_seri()  # Seri porttan veri oku
        if sensor_degerleri is not None:
            # ?lk veriyi al
            gelen_veri = sensor_degerleri[0]  # Gelen verinin ilk eleman?n? al
            pulse = int(map_func(gelen_veri, 0, 1024, 1000, 2000))
            pulse2 = int(map_func(gelen_veri, 1024, 0, 1000, 2000))  # Veriyi haritala (0 ile 1024 aras?nda)
            
            if 512<gelen_veri<=1024:
                pulse = int(map_func(gelen_veri, 0, 1024, 1000, 2000))
                pulse2 = int(map_func(gelen_veri, 1024, 0, 1000, 2000))  # Veriyi haritala (0 ile 1024 aras?nda
                
                for motor in MOTOR_PINS.keys():
            
                    if motor == "motor1":
                        set_motor_hizi(motor, pulse2)
                    
                    else:
                        set_motor_hizi(motor, pulse)

            elif 0<=gelen_veri<512:
                pulse = int(map_func(gelen_veri, 0, 1024, 1000, 2000))
                pulse2 = int(map_func(gelen_veri, 1024, 0, 1000, 2000))  # Veriyi haritala (0 ile 1024 aras?nda
                
                for motor in MOTOR_PINS.keys():
                    if motor == "motor1":
                        set_motor_hizi(motor, pulse2)
                    else:
                        set_motor_hizi(motor, pulse)
                        
                        
                
            else:
                print("else oldu")
                pass                

        time.sleep(0.02)  # 20ms, daha t
except KeyboardInterrupt:
    print("Program durduruldu")
finally:
    if ser and ser.is_open:
        ser.close()  # Seri portu kapat
    # Motorlar? durdur
    for motor in pwm_motorlar.values():
        motor.ChangeDutyCycle(0)  # Motoru durdur
        motor.stop()  # PWM'yi sonland?r
    GPIO.cleanup()  # GPIO temizli?i yap
