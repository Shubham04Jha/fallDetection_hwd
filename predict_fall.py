import joblib
import pandas as pd
import numpy as np

MODEL_FILE = 'more_sensitive_svc.pkl'
FEATURES = ['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'svm_acc']

# Load model
try:
    model = joblib.load(MODEL_FILE)
    print(f" Model '{MODEL_FILE}' loaded successfully.")
except Exception as e:
    print(f" Error loading model: {e}")
    model = None


def predict_fall(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z):
    """Predict whether the given sensor readings indicate a fall."""
    if model is None:
        raise RuntimeError("Model not loaded!")

    # Create input DataFrame
    df = pd.DataFrame([{
        'acc_x': acc_x,
        'acc_y': acc_y,
        'acc_z': acc_z,
        'gyro_x': gyro_x,
        'gyro_y': gyro_y,
        'gyro_z': gyro_z
    }])

    # Compute SVM acceleration feature
    df['svm_acc'] = np.sqrt(df['acc_x']**2 + df['acc_y']**2 + df['acc_z']**2)

    # Predict
    pred = model.predict(df[FEATURES])[0]
    return bool(pred)


if __name__ == "__main__":
    print("\n--- Testing Model ---")

    samples = [
        ("ADL (Resting)", -0.1, 9.7, 0.2, 0.05, 0.1, -0.05),
        ("Fall (Impact)", 15.5, -12.1, 8.9, 5.0, -6.0, 4.0),
        ("Known Sample", -2.020706, 9.672575, -0.354342, 3.166420, -3.186884, -3.760486)
    ]

    for label, ax, ay, az, gx, gy, gz in samples:
        result = predict_fall(ax, ay, az, gx, gy, gz)
        print(f"{label}: {'FALL' if result else 'ADL'}")
