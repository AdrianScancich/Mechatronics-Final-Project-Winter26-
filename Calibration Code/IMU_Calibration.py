import pyb
import time
from imu_driver import IMU

def full_imu_calib():
    i2c = pyb.I2C(1, pyb.I2C.CONTROLLER)
    imu = IMU(i2c)

    sys, gyro, accel, mag = imu.cal_status()
    if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
        print("IMU already calibrated:", (sys, gyro, accel, mag))
        return

    try:
        with open("calibration.txt", "rb") as f:
            coeffs = f.read()
        imu.write_coeff(coeffs)
        time.sleep_ms(200)

        sys, gyro, accel, mag = imu.cal_status()
        print("Calibration loaded:", (sys, gyro, accel, mag))
        return

    except OSError:
        print("Calibration mode entered.")
        while True:
            sys, gyro, accel, mag = imu.cal_status()
            print("CAL:", (sys, gyro, accel, mag))
            if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
                print("Calibration complete.")
                coeffs = imu.cal_coeff()
                with open("calibration.txt", "wb") as f:
                    f.write(coeffs)
                print("Calibration saved to calibration.txt")
                return
            time.sleep(1)

# full_imu_calib() will run this quick task and calibrate the IMU