# imu_utils.py
import time
from simu import imu, SImu  # Import only the imu instance

def initialize_imu():
    """
    Initializes the IMU.  Waits for initial data or times out.
    Prints status messages.  This should be called once at the beginning.
    """
    if imu.gyroUpdCnt == 0 or imu.accUpdCnt == 0:
        print("Waiting for IMU data initialization...")
        imu.setup()  # This will block until data arrives or setup times out


def get_imu_data():
    """
    Safely retrieves the latest IMU data.

    Returns:
        dict: A dictionary with gyro and acc data, timestamps, and intervals,
              or None if data is not yet available.  The dictionary contains:
                - "gyro": A list [x, y, z] of gyroscope readings.
                - "acc": A list [x, y, z] of accelerometer readings.
                - "gyro_time": A datetime object representing the gyro timestamp.
                - "acc_time": A datetime object representing the acc timestamp.
                - "gyro_interval": The time interval between gyro readings (seconds).
                - "acc_interval": The time interval between acc readings (seconds).
    """
    # Removed the lock: with imu.lock:
    if imu.gyroUpdCnt > 0 and imu.accUpdCnt > 0:
        return {
            "gyro": imu.gyro.copy(),
            "acc": imu.acc.copy(),
            "gyro_time": imu.gyroTime,
            "acc_time": imu.accTime,
            "gyro_interval": imu.gyroInterval,
            "acc_interval": imu.accInterval
        }
    else:
        return None