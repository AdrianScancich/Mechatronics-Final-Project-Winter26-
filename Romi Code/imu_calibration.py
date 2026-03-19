import time

def full_imu_calib(imu):
    try:
        with open("calibration.txt", "rb") as f:
            imu.write_coeff(f.read())
        print("Calibration loaded.")
        return
    except OSError:
        print("Calibration mode entered.")

    while True:
        sys, gyro, accel, mag = imu.cal_status()
        print("CAL:", (sys, gyro, accel, mag))
        if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
            with open("calibration.txt", "wb") as f:
                f.write(imu.cal_coeff())
            print("Calibration complete and saved.")
            return
        time.sleep(1)