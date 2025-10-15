#!/usr/bin/env python3
import smbus
import time
import pandas as pd

# ---------------------------------------------
# CONFIGURATION
# ---------------------------------------------
DATA_COLLECTION_DURATION = 60  # seconds (1 minutes)
READ_INTERVAL = 0.1             # seconds between readings
LOG_FILENAME = "fall2_log.csv"

# ---------------------------------------------
# MPU6050 Registers & Addresses
# ---------------------------------------------
DEVICE_ADDRESS = 0x68

# Power management and configuration registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C

# Accelerometer data registers
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40

# Gyroscope data registers
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

# ---------------------------------------------
# Initialize I2C
# ---------------------------------------------
bus = smbus.SMBus(1)

# ---------------------------------------------
# MPU6050 Initialization
# ---------------------------------------------
def MPU_Init():
    bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 0x01)  # wake up
    time.sleep(0.1)
    bus.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 0x07)
    bus.write_byte_data(DEVICE_ADDRESS, CONFIG, 0x03)
    bus.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 0x18)
    bus.write_byte_data(DEVICE_ADDRESS, ACCEL_CONFIG, 0x00)

# ---------------------------------------------
# Read raw 16-bit value
# ---------------------------------------------
def read_raw_data(addr):
    high = bus.read_byte_data(DEVICE_ADDRESS, addr)
    low  = bus.read_byte_data(DEVICE_ADDRESS, addr + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 0x10000
    return value

# ---------------------------------------------
# Sensitivity constants
# ---------------------------------------------
GYRO_SENSITIVITY  = 16.4
ACCEL_SENSITIVITY = 16384.0

# ---------------------------------------------
# Gyro offsets
# ---------------------------------------------
GYRO_OFFSETS = {"x": 0, "y": 0, "z": 0}

def calibrate_gyro_only(samples=100):
    print("Pre-calibrating Gyro... Keep sensor still.")
    gx, gy, gz = 0, 0, 0
    for _ in range(samples):
        gx += read_raw_data(GYRO_XOUT_H)
        gy += read_raw_data(GYRO_YOUT_H)
        gz += read_raw_data(GYRO_ZOUT_H)
        time.sleep(0.01)
    GYRO_OFFSETS["x"] = gx / samples
    GYRO_OFFSETS["y"] = gy / samples
    GYRO_OFFSETS["z"] = gz / samples
    print("Gyro calibration complete:", GYRO_OFFSETS)
    print("waiting 2 secs")
    time.sleep(4)

# ---------------------------------------------
# Get scaled readings
# ---------------------------------------------
def get_scaled_readings():
    Gx = (read_raw_data(GYRO_XOUT_H) - GYRO_OFFSETS["x"]) / GYRO_SENSITIVITY
    Gy = (read_raw_data(GYRO_YOUT_H) - GYRO_OFFSETS["y"]) / GYRO_SENSITIVITY
    Gz = (read_raw_data(GYRO_ZOUT_H) - GYRO_OFFSETS["z"]) / GYRO_SENSITIVITY

    Ax = read_raw_data(ACCEL_XOUT_H) / ACCEL_SENSITIVITY
    Ay = read_raw_data(ACCEL_YOUT_H) / ACCEL_SENSITIVITY
    Az = read_raw_data(ACCEL_ZOUT_H) / ACCEL_SENSITIVITY

    return {"Gx": Gx, "Gy": Gy, "Gz": Gz,
            "Ax": Ax, "Ay": Ay, "Az": Az,
            "timestamp": time.time()}

# ---------------------------------------------
# Main execution
# ---------------------------------------------
def main():
    MPU_Init()
    calibrate_gyro_only()

    data_log = []
    start_time = time.time()
    end_time = start_time + DATA_COLLECTION_DURATION
    print(f"Collecting data for {DATA_COLLECTION_DURATION} seconds...")

    while time.time() < end_time:
        readings = get_scaled_readings()
        data_log.append(readings)
        print(readings, end="\r", flush=True)
        time.sleep(READ_INTERVAL)

    print("\nSaving data to CSV...")
    df = pd.DataFrame(data_log)
    df.to_csv(LOG_FILENAME, index=False)
    print(f"✅ Data saved to '{LOG_FILENAME}'. Auto-complete after {DATA_COLLECTION_DURATION} seconds.")

# ---------------------------------------------
if __name__ == "__main__":
    main()
