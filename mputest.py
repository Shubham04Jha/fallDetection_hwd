'''
Python script to read MPU-6050 Gyroscope AND Accelerometer data at 2Hz (every 0.5 seconds).
NOW INCLUDES CALIBRATION AND DIGITAL LOW PASS FILTER (DLPF) to reduce noise.
'''
import smbus
import time
from time import sleep 
import math

# --- CONFIGURATION CONSTANTS ---
# Increased threshold to filter out high noise floor.
GYRO_THRESHOLD = 7.0 

# Accelerometer threshold (in g's)
ACCEL_THRESHOLD = 1.2 
CALIBRATION_SAMPLES = 10 # Number of readings for calibration (5 seconds at 2Hz)
# -----------------------------

# MPU6050 Register and Address Definitions (Unchanged)
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A # Configuration Register for DLPF
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
DEVICE_ADDRESS = 0x68   

# Calibration Offsets (Global Variables)
GYRO_OFFSETS = [0.0, 0.0, 0.0]
ACCEL_OFFSETS = [0.0, 0.0, 0.0]
GRAVITY_VECTOR = [0.0, 0.0, 0.0]

try:
    bus = smbus.SMBus(1)
except FileNotFoundError:
    print("Error: I2C bus 1 not found. Ensure I2C is enabled in raspi-config.")
    exit()

def MPU_Init():
    """Initializes the MPU-6050 registers."""
    bus.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 0x07)
    bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 0x01)
    
    # --- CHANGE: Enable Digital Low Pass Filter (DLPF) ---
    # Setting 0x03 provides 42Hz Gyro bandwidth and 44Hz Accel bandwidth, 
    # which heavily smooths the data and reduces high-frequency noise.
    bus.write_byte_data(DEVICE_ADDRESS, CONFIG, 0x03) 
    # ----------------------------------------------------
    
    bus.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 0x18) # +/- 2000 deg/s
    bus.write_byte_data(DEVICE_ADDRESS, 0x1C, 0x00)        # +/- 2g
    bus.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 0x01)

def read_raw_data(addr):
    """Reads 16-bit raw data from the MPU-6050 register address."""
    high = bus.read_byte_data(DEVICE_ADDRESS, addr)
    low = bus.read_byte_data(DEVICE_ADDRESS, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value = value - 65536
    return value

def get_raw_readings():
    """Reads raw gyro and accel data, returns scaled values."""
    GYRO_SENSITIVITY = 16.4    # For +/- 2000 deg/s setting (0x18)
    ACCEL_SENSITIVITY = 16384.0 # For +/- 2g setting (0x00)

    Gx = read_raw_data(GYRO_XOUT_H) / GYRO_SENSITIVITY
    Gy = read_raw_data(GYRO_YOUT_H) / GYRO_SENSITIVITY
    Gz = read_raw_data(GYRO_ZOUT_H) / GYRO_SENSITIVITY
    
    Ax = read_raw_data(ACCEL_XOUT_H) / ACCEL_SENSITIVITY
    Ay = read_raw_data(ACCEL_YOUT_H) / ACCEL_SENSITIVITY
    Az = read_raw_data(ACCEL_ZOUT_H) / ACCEL_SENSITIVITY
    
    return [Gx, Gy, Gz, Ax, Ay, Az]

def calibrate_sensor():
    """Performs static calibration and sets global offsets."""
    global GYRO_OFFSETS, ACCEL_OFFSETS, GRAVITY_VECTOR
    
    print("\n--- Calibration Started ---")
    print(f"**PLEASE KEEP THE SENSOR PERFECTLY STILL for {CALIBRATION_SAMPLES * 0.5} seconds.**")
    
    sum_gyro = [0.0, 0.0, 0.0]
    sum_accel = [0.0, 0.0, 0.0]
    
    for i in range(CALIBRATION_SAMPLES):
        readings = get_raw_readings()
        sum_gyro[0] += readings[0]
        sum_gyro[1] += readings[1]
        sum_gyro[2] += readings[2]
        
        sum_accel[0] += readings[3]
        sum_accel[1] += readings[4]
        sum_accel[2] += readings[5]
        
        print(f"Reading {i+1}/{CALIBRATION_SAMPLES}...", end='\r')
        sleep(0.5)

    # Calculate average offsets
    GYRO_OFFSETS[0] = sum_gyro[0] / CALIBRATION_SAMPLES
    GYRO_OFFSETS[1] = sum_gyro[1] / CALIBRATION_SAMPLES
    GYRO_OFFSETS[2] = sum_gyro[2] / CALIBRATION_SAMPLES
    
    ACCEL_OFFSETS[0] = sum_accel[0] / CALIBRATION_SAMPLES
    ACCEL_OFFSETS[1] = sum_accel[1] / CALIBRATION_SAMPLES
    ACCEL_OFFSETS[2] = sum_accel[2] / CALIBRATION_SAMPLES
    
    # Identify the gravity vector (the 1g bias)
    GRAVITY_VECTOR[0] = ACCEL_OFFSETS[0]
    GRAVITY_VECTOR[1] = ACCEL_OFFSETS[1]
    GRAVITY_VECTOR[2] = ACCEL_OFFSETS[2]
    
    print("\nCalibration Complete. Offsets:")
    print(f"  Gyro Offsets: X:{GYRO_OFFSETS[0]:.2f}°/s | Y:{GYRO_OFFSETS[1]:.2f}°/s | Z:{GYRO_OFFSETS[2]:.2f}°/s")
    print(f"  Accel Bias (Gravity): X:{GRAVITY_VECTOR[0]:.2f}g | Y:{GRAVITY_VECTOR[1]:.2f}g | Z:{GRAVITY_VECTOR[2]:.2f}g")
    print("-" * 50)


def get_calibrated_readings():
    """Reads raw data and subtracts the stored offsets."""
    Gx, Gy, Gz, Ax, Ay, Az = get_raw_readings()
    
    # Subtract Gyro drift offset
    Gx -= GYRO_OFFSETS[0]
    Gy -= GYRO_OFFSETS[1]
    Gz -= GYRO_OFFSETS[2]
    
    # Subtract Accel 1g bias (Gravity)
    Ax_linear = Ax - GRAVITY_VECTOR[0]
    Ay_linear = Ay - GRAVITY_VECTOR[1]
    Az_linear = Az - GRAVITY_VECTOR[2]

    return {
        'gyro': [Gx, Gy, Gz],
        'accel': [Ax_linear, Ay_linear, Az_linear],
        'accel_total': [Ax, Ay, Az] 
    }

def process_sensor_data(data):
    """
    Checks if any sensor reading exceeds the defined THRESHOLDS 
    and prints the data only if movement is detected.
    """
    Gx, Gy, Gz = data['gyro']
    Ax_linear, Ay_linear, Az_linear = data['accel']
    Ax_total, Ay_total, Az_total = data['accel_total']
    
    # 1. Check for rotational movement (Calibrated Gyroscope)
    is_rotating = abs(Gx) > GYRO_THRESHOLD or abs(Gy) > GYRO_THRESHOLD or abs(Gz) > GYRO_THRESHOLD
    
    # 2. Check for linear acceleration (Calibrated Accelerometer)
    # Calculate the magnitude of *LINEAR* acceleration (excluding gravity)
    linear_accel_magnitude = math.sqrt(Ax_linear**2 + Ay_linear**2 + Az_linear**2)
    
    # Check acceleration above 0.2g (1.2g - 1.0g)
    is_accelerating = linear_accel_magnitude > (ACCEL_THRESHOLD - 1.0) 

    if is_rotating or is_accelerating:
        # Format the output when movement is detected
        print(f"[{time.strftime('%H:%M:%S')}] **MOVEMENT DETECTED** (Linear Accel: {linear_accel_magnitude:4.2f}g)")
        print(f"  > Gyro (Calibrated): X:{Gx:7.2f}°/s | Y:{Gy:7.2f}°/s | Z:{Gz:7.2f}°/s")
        print(f"  > Accel (Linear): X:{Ax_linear:7.2f}g | Y:{Ay_linear:7.2f}g | Z:{Az_linear:7.2f}g")
    

if __name__ == "__main__":
    MPU_Init()
    
    # Perform Calibration
    calibrate_sensor()
    
    # Define the desired frequency (2Hz = 0.5s interval)
    READ_INTERVAL = 0.5 
    
    print("\nStarting Calibrated Sensor reading loop at 2Hz (press Ctrl+C to stop)...")
    print(f"Gyro will now be centered around 0. Printing only when:")
    print(f" - Gyro reading exceeds +/- {GYRO_THRESHOLD}°/s OR")
    print(f" - Linear Accel exceeds +/- {(ACCEL_THRESHOLD - 1.0):.2f}g.")
    print("-" * 50)
    
    try:
        while True:
            # 1. Read the data
            sensor_data = get_calibrated_readings()
            
            # 2. Pass the data to the separate function
            process_sensor_data(sensor_data)
            
            # 3. Wait for the desired interval
            sleep(READ_INTERVAL)
            
    except KeyboardInterrupt:
        print("\n\nReading stopped by user.")
