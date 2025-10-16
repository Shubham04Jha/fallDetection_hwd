#!/usr/bin/env python3
import smbus
import time
import pandas as pd

# ---------------------------------------------
# CONFIGURATION
# ---------------------------------------------
LABEL = input("Enter label (0 = safe / 1 = fall): ").strip()
DURATION = float(input("Enter collection duration in seconds: ").strip() or 30)
FILENAME = input("Enter output filename (e.g. safe_log.csv): ").strip() or "data_log.csv"

READ_INTERVAL = 0.1  # 10Hz
DEVICE_ADDRESS = 0x68

# MPU6050 Registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

GYRO_SENSITIVITY  = 16.4
ACCEL_SENSITIVITY = 16384.0

bus = smbus.SMBus(1)

# ---------------------------------------------
# Initialize MPU6050
# ---------------------------------------------
def MPU_Init():
    bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 0x00)
    bus.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 0x07)
    bus.write_byte_data(DEVICE_ADDRESS, CONFIG, 0x03)
    bus.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 0x18)
    bus.write_byte_data(DEVICE_ADDRESS, ACCEL_CONFIG, 0x00)

def read_raw_data(addr):
    high = bus.read_byte_data(DEVICE_ADDRESS, addr)
    low  = bus.read_byte_data(DEVICE_ADDRESS, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value

# ---------------------------------------------
# Collect Data
# ---------------------------------------------
def collect_data():
    MPU_Init()
    print("\nCollecting data for", DURATION, "seconds...")
    print("Keep sensor stable if 'safe', or perform motion if 'fall'.")
    print("-" * 50)

    start_time = time.time()
    rows = []

    while (time.time() - start_time) < DURATION:
        Gx = read_raw_data(GYRO_XOUT_H) / GYRO_SENSITIVITY
        Gy = read_raw_data(GYRO_XOUT_H + 2) / GYRO_SENSITIVITY
        Gz = read_raw_data(GYRO_XOUT_H + 4) / GYRO_SENSITIVITY
        Ax = read_raw_data(ACCEL_XOUT_H) / ACCEL_SENSITIVITY
        Ay = read_raw_data(ACCEL_XOUT_H + 2) / ACCEL_SENSITIVITY
        Az = read_raw_data(ACCEL_XOUT_H + 4) / ACCEL_SENSITIVITY

        rows.append({
            "Gx": Gx, "Gy": Gy, "Gz": Gz,
            "Ax": Ax, "Ay": Ay, "Az": Az,
            "label": int(LABEL),
            "timestamp": time.time()
        })

        time.sleep(READ_INTERVAL)

    df = pd.DataFrame(rows)
    df.to_csv(FILENAME, index=False)
    print(f"✅ Saved {len(rows)} samples to {FILENAME}")

# ---------------------------------------------
if __name__ == "__main__":
    collect_data()
