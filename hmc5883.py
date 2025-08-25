import smbus
import time
import json

# I2C adresleri
MPU_ADDR = 0x68
QMC_ADDR = 0x0D

bus = smbus.SMBus(1)

# MPU6050 başlatma fonksiyonu
def mpu_init():
    bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # Wake up MPU6050

# MPU’dan veri okuma
def read_mpu():
    data = bus.read_i2c_block_data(MPU_ADDR, 0x3B, 14)
    accel_x = (data[0] << 8) | data[1]
    accel_y = (data[2] << 8) | data[3]
    accel_z = (data[4] << 8) | data[5]
    temp = (data[6] << 8) | data[7]
    gyro_x = (data[8] << 8) | data[9]
    gyro_y = (data[10] << 8) | data[11]
    gyro_z = (data[12] << 8) | data[13]

    # 2's complement dönüşümü
    accel_x = accel_x - 65536 if accel_x > 32767 else accel_x
    accel_y = accel_y - 65536 if accel_y > 32767 else accel_y
    accel_z = accel_z - 65536 if accel_z > 32767 else accel_z
    gyro_x = gyro_x - 65536 if gyro_x > 32767 else gyro_x
    gyro_y = gyro_y - 65536 if gyro_y > 32767 else gyro_y
    gyro_z = gyro_z - 65536 if gyro_z > 32767 else gyro_z

    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

# QMC5883L başlatma
def qmc_init():
    bus.write_byte_data(QMC_ADDR, 0x0B, 0x01)  # Mode Register: Continuous measurement mode
    bus.write_byte_data(QMC_ADDR, 0x09, 0x1D)  # Control Register: 100Hz, 2G range, OSR=512

# QMC'den veri okuma
def read_qmc():
    data = bus.read_i2c_block_data(QMC_ADDR, 0x00, 6)
    mag_x = (data[1] << 8) | data[0]
    mag_y = (data[3] << 8) | data[2]
    mag_z = (data[5] << 8) | data[4]

    mag_x = mag_x - 65536 if mag_x > 32767 else mag_x
    mag_y = mag_y - 65536 if mag_y > 32767 else mag_y
    mag_z = mag_z - 65536 if mag_z > 32767 else mag_z

    return mag_x, mag_y, mag_z

def main():
    print("MPU ve QMC başlatılıyor...")
    mpu_init()
    qmc_init()
    time.sleep(1)

    mag_x_min = float('inf')
    mag_y_min = float('inf')
    mag_x_max = float('-inf')
    mag_y_max = float('-inf')

    print("Kalibrasyon başlıyor, cihazı 8 şekli hareketle yavaşça döndürün... (15 saniye)")
    start_time = time.time()

    while time.time() - start_time < 15:
        _, _, _, _, _, _ = read_mpu()  # MPU verisi okunuyor ama kalibrasyonda kullanılmıyor (isteğe bağlı)
        mag_x, mag_y, _ = read_qmc()

        if mag_x < mag_x_min:
            mag_x_min = mag_x
        if mag_x > mag_x_max:
            mag_x_max = mag_x

        if mag_y < mag_y_min:
            mag_y_min = mag_y
        if mag_y > mag_y_max:
            mag_y_max = mag_y

        time.sleep(0.05)

    offset_x = (mag_x_max + mag_x_min) / 2
    offset_y = (mag_y_max + mag_y_min) / 2
    scale_x = (mag_x_max - mag_x_min) / 2
    scale_y = (mag_y_max - mag_y_min) / 2

    calibration = {
        "offset_x": offset_x,
        "offset_y": offset_y,
        "scale_x": scale_x,
        "scale_y": scale_y
    }

    with open("mag_calibration.json", "w") as f:
        json.dump(calibration, f)

    print("Kalibrasyon tamamlandı ve dosyaya kaydedildi:")
    print(calibration)

if __name__ == "__main__":
    main()
