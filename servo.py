import RPi.GPIO as GPIO
import time

SERVO_PIN = 18  # PWM çıkışı için pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(7.5)  # orta konum

# Başlangıç açısı
current_angle = 90  

def set_servo_angle(angle):
    duty = 2.5 + (angle / 18)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.2)

try:
    while True:
       
        komut = input("Kumandadan gelen (1 veya 0): ").strip()

        if komut == "1":
            current_angle += 10   # + yönde hareket
            if current_angle > 180:
                current_angle = 180
            set_servo_angle(current_angle)

        elif komut == "0":
            current_angle -= 10   # - yönde hareket
            if current_angle < 0:
                current_angle = 0
            set_servo_angle(current_angle)

        print("Servo açısı:", current_angle)

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()