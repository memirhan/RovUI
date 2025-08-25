import smbus
import time
import math
import json

MPU_ADDR = 0x68
QMC_ADDR = 0x0D

bus = smbus.SMBus(1)

def mpu_init():
    bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # Wake up MPU

def read_mpu():
    data = bus.read_i2c_block_data(MPU_ADDR, 0x3B, 14)
    accel_x = (data[0] << 8) | data[1]
    accel_y = (data[2] << 8) | data[3]
    accel_z = (data[4] << 8) | data[5]
    temp = (data[6] << 8) | data[7]
    gyro_x = (data[8] << 8) | data[9]
    gyro_y = (data[10] << 8) | data[11]
    gyro_z = (data[12] << 8) | data[13]

    accel_x = accel_x - 65536 if accel_x > 32767 else accel_x
    accel_y = accel_y - 65536 if accel_y > 32767 else accel_y
    accel_z = accel_z - 65536 if accel_z > 32767 else accel_z
    gyro_x = gyro_x - 65536 if gyro_x > 32767 else gyro_x
    gyro_y = gyro_y - 65536 if gyro_y > 32767 else gyro_y
    gyro_z = gyro_z - 65536 if gyro_z > 32767 else gyro_z

    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

def qmc_init():
    bus.write_byte_data(QMC_ADDR, 0x0B, 0x01)
    bus.write_byte_data(QMC_ADDR, 0x09, 0x1D)

def read_qmc():
    data = bus.read_i2c_block_data(QMC_ADDR, 0x00, 6)
    mag_x = (data[1] << 8) | data[0]
    mag_y = (data[3] << 8) | data[2]
    mag_z = (data[5] << 8) | data[4]

    mag_x = mag_x - 65536 if mag_x > 32767 else mag_x
    mag_y = mag_y - 65536 if mag_y > 32767 else mag_y
    mag_z = mag_z - 65536 if mag_z > 32767 else mag_z

    return mag_x, mag_y, mag_z

def calculate_heading(mx, my):
    heading_rad = math.atan2(my, mx)
    heading_deg = math.degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg

def main():
    print("MPU ve QMC başlatılıyor...")
    mpu_init()
    qmc_init()
    time.sleep(1)

    try:
        with open("mag_calibration.json") as f:
            cal = json.load(f)
        print("Kalibrasyon dosyası yüklendi.")
    except FileNotFoundError:
        print("Kalibrasyon dosyası bulunamadı! Çıkılıyor.")
        return

    while True:
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = read_mpu()
        mag_x, mag_y, mag_z = read_qmc()

        # Kalibrasyon düzeltmeleri
        mx_corr = (mag_x - cal["offset_x"]) / cal["scale_x"]
        my_corr = (mag_y - cal["offset_y"]) / cal["scale_y"]

        heading = calculate_heading(mx_corr, my_corr)

        print(f"Accel X:{accel_x} Y:{accel_y} Z:{accel_z}")
        print(f"Gyro X:{gyro_x} Y:{gyro_y} Z:{gyro_z}")
        print(f"Mag  X:{mag_x} Y:{mag_y} Z:{mag_z}")
        print(f"Heading: {heading:.2f}°")
        print("-----------------------------")
        time.sleep(0.5)

if __name__ == "__main__":
    main()
