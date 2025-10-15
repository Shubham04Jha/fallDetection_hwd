from mpu6050 import mpu6050
import time

sensor = mpu6050(0x68)  # I2C address

print("Reading MPU6050 data... Press Ctrl+C to stop.\n")

try:
    while True:
        accel_data = sensor.get_accel_data()
        gyro_data = sensor.get_gyro_data()
        temp = sensor.get_temp()

        print(f"Accelerometer: x={accel_data['x']:.2f}, y={accel_data['y']:.2f}, z={accel_data['z']:.2f}")
        print(f"Gyroscope:     x={gyro_data['x']:.2f}, y={gyro_data['y']:.2f}, z={gyro_data['z']:.2f}")
        print(f"Temperature:   {temp:.2f}°C")
        print("-" * 50)
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopped.")