import RPi.GPIO as GPIO
import time
import serial

# Seri portu baÅlat
try:
    ser = serial.Serial('/dev/ttyVirtualArduino', 9600, timeout=0.1)  # Non-blocking
    time.sleep(0.1)
except serial.SerialException:
    print("Seri port bulunamadÄ± /dev/ttyUSB0")
    ser = None

# Motor pinleri
MOTOR_PINS = {
    "motor1": 26, "motor2": 19, "motor3": 13, "motor4": 6,  # SaÄ/sol motorlar
    "motor5": 21, "motor6": 20, "motor7": 16, "motor8": 5    # YukarÄ±/aÅaÄÄ± motorlar
}

GPIO.setmode(GPIO.BCM)

# Motor pinlerini giriÅ/Ã§Ä±kÄ±Å olarak ayarla
for motor in MOTOR_PINS.values():
    GPIO.setup(motor, GPIO.OUT)

# PWM baÅlat
pwm_motorlar = {}
for motor, pin in MOTOR_PINS.items():
    pwm_motorlar[motor] = GPIO.PWM(pin, 50)  # 50Hz
    pwm_motorlar[motor].start(0)

# Motor hÄ±zÄ±nÄ± ayarlamak iÃ§in fonksiyon
def set_motor_hizi(motor_adi, pulse):
    duty = pulse / 20000 * 100  
    duty = max(0, min(duty, 100))  # Duty %0 ile %100 arasÄ±nda kalmalÄ±
    pwm_motorlar[motor_adi].ChangeDutyCycle(duty)
    print(f"{motor_adi} hÄ±zÄ±: {pulse} us (duty: {duty:.2f}%)")

# Kalibrasyon fonksiyonu
def kalibrasyon():
    print("Motor kalibrasyonu baÅlatÄ±lÄ±yor...")
    # Her motoru kalibre et
    for motor in MOTOR_PINS.keys():
        set_motor_hizi(motor, 1500)  # Motoru hÄ±zlÄ± baÅlat
    time.sleep(2)
    print("Kalibrasyon tamamlandÄ±")

# Veriyi belirli bir aralÄ±Äa haritalama fonksiyonu
def map_func(deger, giris_min, giris_max, cikis_min, cikis_max):
    return (deger - giris_min) * (cikis_max - cikis_min) / (giris_max - giris_min) + cikis_min

# Seri porttan veri okuma fonksiyonu
def oku_seri():
    if ser and ser.in_waiting > 0:
        try:
            data = ser.readline().decode('utf-8').strip()
            su = data.split(",")  # Gelen veriyi virgÃ¼lle ayÄ±r
            
            print(data)
            return [int(x) for x in su]  # Verileri liste olarak dÃ¶ndÃ¼r
        except Exception as e:
            print(f"Hata oluÅtu: {e}")
            return None
    return None

# Ana program
try:
    kalibrasyon()  # Kalibrasyonu baÅlat
    first_value = None 
    list = []
    flag = False
    stop_flag = False
    while True:
        sensor_degerleri = oku_seri()  # Seri porttan veri oku
        if sensor_degerleri is not None:
            # Ä°lk veriyi al
            
            
            gelen_sag = sensor_degerleri[0]  # Gelen verinin ilk elemanÄ±nÄ± al
            gelen_veri_ileri = sensor_degerleri[1]  # SaÄ/sol motor iÃ§in gelen veri
            gelen_veri_yukari = sensor_degerleri[2]  # YukarÄ±/aÅaÄÄ± motor iÃ§in gelen veri
            list.append(gelen_sag)  # Gelen veriyi listeye ekle
            print(f"Gelen veri: {gelen_sag}")

            # Ä°lk veri geldiÄinde, onu baÅlangÄ±Ã§ deÄeri olarak al
            if first_value is None:
                first_value = gelen_sag
                print(f"Ä°lk veri: {first_value}")
            
            # EÄer ilk farklÄ± veri geldiyse, flag'Ä± True yap
            if gelen_sag != first_value and not flag:
                print(f"Ä°lk farklÄ± veri geldi: {gelen_sag}")
                flag = True

            # EÄer bir kez farklÄ± veri geldiyse ve Åimdi 512 geldiyse, motorlarÄ± durdur
            if flag and gelen_sag == 512:
                print("512 geldi, motorlar durduruluyor...")
                stop_flag = True  # MotorlarÄ± durdurmak iÃ§in flag'Ä± True yap

            # EÄer stop_flag True ise motorlarÄ± durdur, ancak tamamen kapatmak yerine hÄ±zÄ±nÄ± sÄ±fÄ±rla
            if stop_flag:
                for motor in pwm_motorlar.values():
                    motor.ChangeDutyCycle(1)  # MotorlarÄ± durdur (hÄ±zlarÄ±nÄ± sÄ±fÄ±rla)
                print("Motorlar durdu, ama kaydedilen verilerle motorlar hareket etmeye devam edecek.")
            
            # Gelen veriye gÃ¶re motor hÄ±zÄ±nÄ± ayarla, ancak stop_flag False olduÄunda motorlarÄ± hareket ettir
            if gelen_sag != 512:  # EÄer gelen veri 512 deÄilse, motorlarÄ± tekrar Ã§alÄ±ÅtÄ±r
                stop_flag = False  # MotorlarÄ± tekrar hareket ettir

            
                            
                        
            else:
                time.sleep(0.02) 

            # Veriyi haritala (0 ile 1024 arasÄ±nda)
            ileri_pluse = int(map_func(gelen_veri_ileri, 0, 1024, 1000, 2000))
            ileri_pluse2 = int(map_func(gelen_veri_ileri, 1024, 0, 1000, 2000))
            
            yukari_pulse = int(map_func(gelen_veri_yukari, 0, 1024, 1000, 2000))
            yukari_pulse2 = int(map_func(gelen_veri_yukari, 1024, 0, 1000, 2000))

            # MotorlarÄ± bir for dÃ¶ngÃ¼sÃ¼ ile kontrol et
            for idx, motor in enumerate(MOTOR_PINS.keys()):
                if idx < 4:  
                    if motor == "motor4":
                        set_motor_hizi(motor, ileri_pluse2)
                    else:
                        set_motor_hizi(motor, ileri_pluse)
                        
                else:  # YukarÄ±/aÅaÄÄ± motorlar
                    if motor == "motor5":
                        set_motor_hizi(motor, yukari_pulse2)
                        
                    elif motor == "motor6":
                        set_motor_hizi(motor, yukari_pulse2)
                        
                    else:
                        set_motor_hizi(motor, yukari_pulse)
                        
                if not stop_flag and idx < 4:
                    sag_pulse = int(map_func(gelen_sag, 0, 1024, 1000, 2000))
                    sag_pulse2 = int(map_func(gelen_sag, 1024, 0, 1000, 2000))  # Veriyi haritala (0 ile 1024 arasÄ±nda)
                    
                    if 512 < gelen_sag <= 1024:
                        print(list)
                        for motor in MOTOR_PINS.keys():
                            if motor == "motor1":
                                set_motor_hizi(motor, sag_pulse2)
                            else:
                                set_motor_hizi(motor, sag_pulse)

                    elif 0 <= gelen_sag < 512:
                        for motor in MOTOR_PINS.keys():
                            if motor == "motor1":
                                set_motor_hizi(motor, sag_pulse2)
                            else:
                                set_motor_hizi(motor, sag_pulse)


        time.sleep(0.02)  # 20ms, daha hÄ±zlÄ± veri almak iÃ§in

except KeyboardInterrupt:
    print("Program durduruldu")
finally:
    if ser and ser.is_open:
        ser.close()  # Seri portu kapat
    # MotorlarÄ± durdur
    for motor in pwm_motorlar.values():
        motor.ChangeDutyCycle(0)  # Motoru durdur
        motor.stop()  # PWM'yi sonlandÄ±r
    GPIO.cleanup()  # GPIO temizliÄi yap
