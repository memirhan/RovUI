import pigpio
import time

# GPIO pinlerini ESC'lere atıyoruz
MOTOR_PINS = {
    "motor1": 18,
    "motor2": 19,
    "motor3": 20,
    "motor4": 21,
    "motor5": 22,
    "motor6": 23,
    "motor7": 24,
    "motor8": 25
}

pi = pigpio.pi()
if not pi.connected:
    exit("pigpio daemon başlatılmamış!")

# ESC'leri başlat (pulsewidth = 0)
pi.set_servo_pulsewidth(MOTOR_PINS["motor1"], 0)
pi.set_servo_pulsewidth(MOTOR_PINS["motor2"], 0)
pi.set_servo_pulsewidth(MOTOR_PINS["motor3"], 0)
pi.set_servo_pulsewidth(MOTOR_PINS["motor4"], 0)
pi.set_servo_pulsewidth(MOTOR_PINS["motor5"], 0)
pi.set_servo_pulsewidth(MOTOR_PINS["motor6"], 0)
pi.set_servo_pulsewidth(MOTOR_PINS["motor7"], 0)
pi.set_servo_pulsewidth(MOTOR_PINS["motor8"], 0)

# Her motor için ayrı kontrol fonksiyonu
def set_motor_speed(motor_name, pulse):
    """
    motor_name: motor1, motor2, ...
    pulse: 1000-2000 mikrosaniye arası
    """
    pin = MOTOR_PINS[motor_name]
    pi.set_servo_pulsewidth(pin, pulse)
    print(f"{motor_name} hızı: {pulse}")

# Kalibrasyon fonksiyonu (tüm motorlar ayrı ayrı)
def calibrate_motors():
    print("Tüm motorlar kalibrasyonu başlatılıyor...")

    # Max throttle
    set_motor_speed("motor1", 2000)
    set_motor_speed("motor2", 2000)
    set_motor_speed("motor3", 2000)
    set_motor_speed("motor4", 2000)
    set_motor_speed("motor5", 2000)
    set_motor_speed("motor6", 2000)
    set_motor_speed("motor7", 2000)
    set_motor_speed("motor8", 2000)
    time.sleep(2)

    # Min throttle
    set_motor_speed("motor1", 1000)
    set_motor_speed("motor2", 1000)
    set_motor_speed("motor3", 1000)
    set_motor_speed("motor4", 1000)
    set_motor_speed("motor5", 1000)
    set_motor_speed("motor6", 1000)
    set_motor_speed("motor7", 1000)
    set_motor_speed("motor8", 1000)
    time.sleep(2)

    print("Kalibrasyon tamamlandı ✅")

# Motorları kapatma
def stop_motors():
    set_motor_speed("motor1", 0)
    set_motor_speed("motor2", 0)
    set_motor_speed("motor3", 0)
    set_motor_speed("motor4", 0)
    set_motor_speed("motor5", 0)
    set_motor_speed("motor6", 0)
    set_motor_speed("motor7", 0)
    set_motor_speed("motor8", 0)
    print("Tüm motorlar durduruldu.")

# -----------------------------
# Örnek kullanım
# -----------------------------
try:
    calibrate_motors()

    # Her motoru farklı hızlarda çalıştırma (manuel olarak ayrı ayrı)
    set_motor_speed("motor1", 1200)
    set_motor_speed("motor2", 1300)
    set_motor_speed("motor3", 1400)
    set_motor_speed("motor4", 1500)
    set_motor_speed("motor5", 1600)
    set_motor_speed("motor6", 1700)
    set_motor_speed("motor7", 1800)
    set_motor_speed("motor8", 1900)

    time.sleep(5)  # 5 saniye çalıştır

    stop_motors()

finally:
    stop_motors()
    pi.stop()