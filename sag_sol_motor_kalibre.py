import RPi.GPIO as GPIO
import time
import serial

# Seri portu baÃat
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

    first_sag = None
    first_ileri = None
    first_yukari = None
     
    list_sag = []
    list_ileri = []
    list_yukari = []
    
    flag_sag= False
    flag_ileri = False
    flag_yukari = False
    
    stop_sag = False
    stop_ileri = False
    stop_yukari= False
    
    while True:
        sensor_degerleri = oku_seri()  # Seri porttan veri oku
        if sensor_degerleri is not None:
            gelen_sag = sensor_degerleri[0]  # Gelen verinin ilk elemanÃÂ±nÃÂ± al
            gelen_veri_ileri = sensor_degerleri[1]  # SaÃ/sol motor iÃÂ§in gelen veri
            gelen_veri_yukari = sensor_degerleri[2]  # YukarÃÂ±/aÃÃÂ± motor iÃÂ§in gelen veri
            list_sag.append(gelen_sag)
            list_ileri.append(gelen_veri_yukari)  # Gelen veriyi listeye ekle
            list_yukari.append(gelen_veri_yukari)  # Gelen veriyi listeye ekle
        

            # ÃÂ°lk veri geldiÃinde, onu baÃlangÃÂ±ÃÂ§ deeri olarak al
                
                
            if first_sag is None:
                first_sag = gelen_sag
                print(f"ÃÂ°lk veri: {first_sag}")
                
            if first_ileri is None:
                first_ileri = gelen_veri_ileri
                print(f"ÃÂ°lk veri: {first_ileri}")
                
            if first_yukari is None:
                first_yukari = gelen_veri_yukari
                print(f"ÃÂ°lk veri: {gelen_veri_yukari}")
            
            # EÃer ilk farklÃÂ± veri geldiyse, flag'ÃÂ± True yap
                
            if gelen_sag != first_sag and not flag_sag:
                print(f"ÃÂ°lk farklÃÂ± veri geldi: {gelen_sag}")
                flag_sag = True
                
            if gelen_veri_ileri != first_ileri and not flag_ileri:
                print(f"ÃÂ°lk farklÃÂ± veri geldi: {gelen_veri_ileri}")
                flag_ileri = True
                
            if gelen_veri_yukari != first_yukari and not flag_yukari:
                print(f"ÃÂ°lk farklÃÂ± veri geldi: {gelen_veri_yukari}")
                flag_yukari = True

                
            if flag_sag and gelen_sag == 512:
                print("512 geldi, motorlar durduruluyor...")
                stop_sag = True
                
            if flag_ileri and gelen_veri_ileri == 512:
                print("512 geldi, motorlar durduruluyor...")
                stop_ileri = True
                
                
            if flag_yukari and gelen_veri_yukari == 512:
                print("512 geldi, motorlar durduruluyor...")
                stop_yukari = True

                
            if stop_yukari:
                for motor_name in ["motor1", "motor2", "motor3", "motor4"]:  # Yukar? motorlar
                    pwm_motorlar[motor_name].ChangeDutyCycle(1)
                print("Yukewar? motorlar durdu.")
                
            if stop_yukari:
                for motor_name in ["motor5", "motor6", "motor7", "motor8"]:  # Yukar? motorlar
                    pwm_motorlar[motor_name].ChangeDutyCycle(1)
                print("Yukar? motorlar durdu.")
            
                
            if gelen_sag != 512:
                stop_sag = False
                
            if gelen_veri_ileri != 512:
                stop_ileri = False

                
            if gelen_veri_yukari != 512:
                stop_yukari = False
                
                
                
            print("Ileri Stop: ", stop_ileri)
            print("Sag Stop: ", stop_sag)
 
 
            for idx, motor in enumerate(MOTOR_PINS.keys()):
                if not stop_yukari and idx < 4:
                    yukari_pulse = int(map_func(gelen_veri_yukari, 0, 1024, 1000, 2000))
                    yukari_pulse2 = int(map_func(gelen_veri_yukari, 1024, 0, 1000, 2000))
                        
                    if 512 < gelen_veri_yukari <= 1024:
                        for motor in MOTOR_PINS.keys():
                            if motor == "motor5":
                                set_motor_hizi(motor, yukari_pulse2)
                                    
                            if motor == "motor6":
                                set_motor_hizi(motor, yukari_pulse2)
                                    
                            if motor == "motor7":
                                set_motor_hizi(motor, yukari_pulse)
                                    
                            if motor == "motor8":
                                set_motor_hizi(motor, yukari_pulse)
                            else:
                                pass

                    if 0 <= gelen_veri_yukari < 512:
                        for motor in MOTOR_PINS.keys():
                            if motor == "motor5":
                                set_motor_hizi(motor, yukari_pulse2)
                                    
                            if motor == "motor6":
                                set_motor_hizi(motor, yukari_pulse2)
                                    
                            if motor == "motor7":
                                set_motor_hizi(motor, yukari_pulse)
                                    
                            if motor == "motor8":
                                    set_motor_hizi(motor, yukari_pulse)
                            else:
                                pass
                                
            for idx, motor in enumerate(MOTOR_PINS.keys()):
                if not stop_ileri and idx < 4:
                    ileri_pluse = int(map_func(gelen_veri_ileri, 0, 1024, 1000, 2000))
                    ileri_pluse2 = int(map_func(gelen_veri_ileri, 1024, 0, 1000, 2000))
                        
                    if 512 < gelen_veri_ileri <= 1024:
                        for motor in MOTOR_PINS.keys():
                            if motor == "motor4":
                                set_motor_hizi(motor, ileri_pluse2)
                                    
                            if motor == "motor1":
                                set_motor_hizi(motor, ileri_pluse)
                                    
                            if motor == "motor2":
                                set_motor_hizi(motor, ileri_pluse)
                                    
                            if motor == "motor3":
                                set_motor_hizi(motor, ileri_pluse)
                            else:
                                pass
                                
                    if 0 <= gelen_veri_ileri < 512:
                        for motor in MOTOR_PINS.keys():
                            if motor == "motor4":
                                set_motor_hizi(motor, ileri_pluse2)
                                    
                            if motor == "motor1":
                                set_motor_hizi(motor, ileri_pluse)
                                    
                            if motor == "motor2":
                                set_motor_hizi(motor, ileri_pluse)
                                    
                            if motor == "motor3":
                                    set_motor_hizi(motor, ileri_pluse)
                            else:
                                pass
                                
            for idx, motor in enumerate(MOTOR_PINS.keys()):
                if not stop_sag and idx < 4:
                    sag_pulse = int(map_func(gelen_sag, 0, 1024, 1000, 2000))
                    sag_pulse2 = int(map_func(gelen_sag, 1024, 0, 1000, 2000))  # Veriyi haritala (0 ile 1024 arasÃÂ±nda)
                        
                    if 512 < gelen_sag <= 1024:
                        for motor in MOTOR_PINS.keys():
                            if motor == "motor1":
                                set_motor_hizi(motor, sag_pulse2)
                                    
                            if motor == "motor4":
                                set_motor_hizi(motor, sag_pulse)
                                    
                            if motor == "motor2":
                                set_motor_hizi(motor, sag_pulse)
                                    
                            if motor == "motor3":
                                set_motor_hizi(motor, sag_pulse)
                            else:
                                pass
                                
                    if 0 <= gelen_sag < 512:
                        for motor in MOTOR_PINS.keys():
                            if motor == "motor1":
                                set_motor_hizi(motor, sag_pulse2)
                                    
                            if motor == "motor4":
                                set_motor_hizi(motor, sag_pulse)
                                    
                            if motor == "motor2":
                                set_motor_hizi(motor, sag_pulse)
                                    
                            if motor == "motor3":
                                    set_motor_hizi(motor, sag_pulse)
                            else:
                                pass
                                
                            
                            

        time.sleep(0.02)

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
