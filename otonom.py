import time, math
import RPi.GPIO as GPIO


HIZ_MS = 0.30                   
GOREV_DERINLIK_M = 3.0           
GUVENLIK_KATSAYISI = 1.10        
PWM_FREKANS = 50

# pinler
MOTOR_PINS = {}

ILERI_GUC = 0.55

KP, KI, KD = 0.9, 0.05, 0.12

GPIO.setmode(GPIO.BCM)
GPIO.setup("motor1", GPIO.OUT)
GPIO.setup("motor1", GPIO.OUT)

pwm_sol = GPIO.PWM("motor1", PWM_FREKANS)
pwm_sag = GPIO.PWM("motor1", PWM_FREKANS)
pwm_sol.start(7.5)
pwm_sag.start(7.5)

def esc_ayarla(pwm_obj, deger):

    deger = max(-1.0, min(1.0, deger))
    pulse_us = 1500 + 500 * deger
    duty = (pulse_us / 20000.0) * 100.0
    pwm_obj.ChangeDutyCycle(duty)

def diferansiyel_itki(ileri, donus):
    """
    ileri: 0..1 (ileri güç)
    donus: -1..1 (sağa/sola dönüş farkı, +sağ)
    """
    sol = ileri - donus
    sag = ileri + donus
    maxmutlak = max(1.0, abs(sol), abs(sag))
    sol /= maxmutlak
    sag /= maxmutlak
    esc_ayarla(pwm_sol, sol)
    esc_ayarla(pwm_sag, sag)

def iticileri_durdur():
    esc_ayarla(pwm_sol, 0.0)
    esc_ayarla(pwm_sag, 0.0)


def mesafe_hesapla(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p = math.pi/180.0
    dlat = (lat2-lat1)*p
    dlon = (lon2-lon1)*p
    a = math.sin(dlat/2)**2 + math.cos(lat1*p)*math.cos(lat2*p)*math.sin(dlon/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R*c

def yon_hesapla(lat1, lon1, lat2, lon2):
    p = math.pi/180.0
    y = math.sin((lon2-lon1)*p) * math.cos(lat2*p)
    x = math.cos(lat1*p)*math.sin(lat2*p) - math.sin(lat1*p)*math.cos(lat2*p)*math.cos((lon2-lon1)*p)
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360) % 360

def aci_sar(deger):
    # -180..+180 arası
    return (deger + 180) % 360 - 180

def imu_yon_deg():
    return imu_okuma_fonksiyonun()

def derinlige_in(hedef_m):
    # Basınç sensörü + PID (sen dolduracaksın)
    pass

def yuzeye_cik():
    iticileri_durdur()

def imu_okuma_fonksiyonun():
    pass

    #imu kodu yazılacak

def otonom_gorev(bas_lat, bas_lon, hedef_lat, hedef_lon):
    mesafe_m = mesafe_hesapla(bas_lat, bas_lon, hedef_lat, hedef_lon)
    hedef_yon = yon_hesapla(bas_lat, bas_lon, hedef_lat, hedef_lon)
    temel_sure = mesafe_m / HIZ_MS
    gorev_suresi = temel_sure * GUVENLIK_KATSAYISI

    print(f"Mesafe: {mesafe_m:.1f} m")
    print(f"Hedef yön: {hedef_yon:.1f}°")
    print(f"Süre: {temel_sure:.1f} sn (güvenlikli: {gorev_suresi:.1f} sn)")

    derinlige_in(GOREV_DERINLIK_M)

    toplam_hata = 0.0
    onceki_hata = 0.0
    onceki_zaman = time.time()
    baslangic_zaman = onceki_zaman

    try:
        while True:
            simdi = time.time()
            dt = max(1e-3, simdi - onceki_zaman)

            # Hata hesabı
            gercek_yon = imu_yon_deg()
            hata = aci_sar(hedef_yon - gercek_yon)

            toplam_hata += hata * dt
            turev = (hata - onceki_hata) / dt

            donus_komutu = KP*hata + KI*toplam_hata + KD*turev
            donus_komutu = max(-1.0, min(1.0, donus_komutu/45.0))

            diferansiyel_itki(ILERI_GUC, donus_komutu)

            onceki_hata = hata
            onceki_zaman = simdi

            if (simdi - baslangic_zaman) >= gorev_suresi:
                print("Süre doldu, yüzeye çıkılıyor...")
                break

            time.sleep(0.05)

    finally:
        yuzeye_cik()
        pwm_sol.stop()
        pwm_sag.stop()
        GPIO.cleanup()


# otonom_gorev(baslat_lat, baslat_lon, hedef_lat, hedef_lon)