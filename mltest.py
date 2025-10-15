import smbus
import time
from time import sleep 
import math
import joblib
import pandas as pd
import numpy as np

# --- ML Model Configuration ---
MODEL_FILE = 'fall_detection_model_LITE.joblib'
# NOTE: The model expects TOTAL acceleration (acc_x, acc_y, acc_z) and calibrated gyro
FEATURES = ['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'svm_acc']
# ------------------------------

# --- MPU6050 Configuration ---
DEVICE_ADDRESS = 0x68    
CALIBRATION_SAMPLES = 10 
READ_INTERVAL = 0.5 # 2Hz reading frequency

# MPU6050 Register Definitions
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A 
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
# -----------------------------

# Calibration Offsets (Global Variables)
GYRO_OFFSETS = [0.0, 0.0, 0.0]

# Initialize I2C Bus and Load Model
try:
    bus = smbus.SMBus(1)
except FileNotFoundError:
    print("Error: I2C bus 1 not found. Ensure I2C is enabled in raspi-config.")
    exit()

try:
    model = joblib.load(MODEL_FILE)
    print(f"✅ Model '{MODEL_FILE}' loaded successfully.")
except Exception as e:
    print(f"❌ Error loading model: {e}")
    model = None

# ----------------------------------------------------------------------
# --- MPU6050 FUNCTIONS ---
# ----------------------------------------------------------------------

def MPU_Init():
    """Initializes the MPU-6050 registers."""
    bus.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 0x07) # Sample rate 1kHz/(7+1) = 125Hz
    bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 0x01) # PLL with X-axis gyro reference
    bus.write_byte_data(DEVICE_ADDRESS, CONFIG, 0x03)     # DLPF (Digital Low Pass Filter)
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

def get_scaled_readings():
    """Reads raw gyro and accel data, returns scaled values."""
    GYRO_SENSITIVITY = 16.4     # For +/- 2000 deg/s setting (0x18)
    ACCEL_SENSITIVITY = 16384.0 # For +/- 2g setting (0x00)

    Gx = read_raw_data(GYRO_XOUT_H) / GYRO_SENSITIVITY
    Gy = read_raw_data(GYRO_YOUT_H) / GYRO_SENSITIVITY
    Gz = read_raw_data(GYRO_ZOUT_H) / GYRO_SENSITIVITY
    
    Ax = read_raw_data(ACCEL_XOUT_H) / ACCEL_SENSITIVITY
    Ay = read_raw_data(ACCEL_YOUT_H) / ACCEL_SENSITIVITY
    Az = read_raw_data(ACCEL_ZOUT_H) / ACCEL_SENSITIVITY
    
    # Returns [Gx, Gy, Gz, Ax, Ay, Az]
    return [Gx, Gy, Gz, Ax, Ay, Az]

def calibrate_gyro_only():
    """Performs static calibration for gyro only."""
    global GYRO_OFFSETS
    
    print("\n--- Gyro Calibration Started ---")
    print(f"**KEEP THE SENSOR PERFECTLY STILL for {CALIBRATION_SAMPLES * 0.5} seconds.**")
    
    sum_gyro = [0.0, 0.0, 0.0]
    
    for i in range(CALIBRATION_SAMPLES):
        readings = get_scaled_readings()
        sum_gyro[0] += readings[0]
        sum_gyro[1] += readings[1]
        sum_gyro[2] += readings[2]
        print(f"Reading {i+1}/{CALIBRATION_SAMPLES}...", end='\r')
        sleep(0.5)

    # Calculate average offsets
    GYRO_OFFSETS[0] = sum_gyro[0] / CALIBRATION_SAMPLES
    GYRO_OFFSETS[1] = sum_gyro[1] / CALIBRATION_SAMPLES
    GYRO_OFFSETS[2] = sum_gyro[2] / CALIBRATION_SAMPLES
    
    print("\nCalibration Complete. Gyro Offsets:")
    print(f"  X:{GYRO_OFFSETS[0]:.2f}°/s | Y:{GYRO_OFFSETS[1]:.2f}°/s | Z:{GYRO_OFFSETS[2]:.2f}°/s")
    print("-" * 50)

def get_model_input():
    """Reads raw data, calibrates gyro, and returns data in a flat list."""
    Gx_raw, Gy_raw, Gz_raw, Ax_total, Ay_total, Az_total = get_scaled_readings()
    
    # Subtract Gyro drift offset
    Gx_calibrated = Gx_raw - GYRO_OFFSETS[0]
    Gy_calibrated = Gy_raw - GYRO_OFFSETS[1]
    Gz_calibrated = Gz_raw - GYRO_OFFSETS[2]
    
    # The model expects TOTAL acceleration, so we use Ax_total, Ay_total, Az_total
    return [
        Ax_total, Ay_total, Az_total, 
        Gx_calibrated, Gy_calibrated, Gz_calibrated
    ]

# ----------------------------------------------------------------------
# --- ML Model Prediction Function ---
# ----------------------------------------------------------------------

def predict_fall(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z):
    """Predict whether the given sensor readings indicate a fall (True=Fall, False=ADL)."""
    if model is None:
        return None

    # Create input DataFrame
    df = pd.DataFrame([{
        'acc_x': acc_x,
        'acc_y': acc_y,
        'acc_z': acc_z,
        'gyro_x': gyro_x,
        'gyro_y': gyro_y,
        'gyro_z': gyro_z
    }])

    # Compute SVM acceleration feature (based on total acceleration)
    df['svm_acc'] = np.sqrt(df['acc_x']**2 + df['acc_y']**2 + df['acc_z']**2)

    # Predict (0=ADL, 1=Fall)
    pred = model.predict(df[FEATURES])[0]
    return bool(pred)

# ----------------------------------------------------------------------
# --- Main Execution Loop ---
# ----------------------------------------------------------------------

if __name__ == "__main__":
    MPU_Init()
    calibrate_gyro_only()
    
    print("\nStarting MPU-6050 Fall Detection Loop (press Ctrl+C to stop)...")
    print(f"Reading sensor and predicting every {READ_INTERVAL} seconds.")
    print("-" * 50)
    
    try:
        while True:
            # 1. Get calibrated/scaled readings for the model
            data = get_model_input()
            ax, ay, az, gx, gy, gz = data
            
            # 2. Predict
            if model:
                is_fall = predict_fall(ax, ay, az, gx, gy, gz)
                if not is_fall:
                    continue
                prediction_label = 'FALL DETECTED ' if is_fall else '✅ SAFE (ADL)'
                
                # 3. Print result and data
                print(f"[{time.strftime('%H:%M:%S')}] **{prediction_label}**")
                print(f"  Accel (Total): X:{ax:6.2f}g | Y:{ay:6.2f}g | Z:{az:6.2f}g")
                print(f"  Gyro (Calib.): X:{gx:6.2f}°/s | Y:{gy:6.2f}°/s | Z:{gz:6.2f}°/s")
                print("-" * 50)
            else:
                print(f"[{time.strftime('%H:%M:%S')}] Model not loaded, skipping prediction.")
                print(f"  Data: Ax={ax:6.2f}g, Ay={ay:6.2f}g, Az={az:6.2f}g")
                print("-" * 50)
                break

            # 4. Wait for the desired interval
            sleep(READ_INTERVAL)
            
    except KeyboardInterrupt:
        print("\n\nReading stopped by user.")
