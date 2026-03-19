import pyb
import time

class IMU:
    '''A class to setup and read data from the inertial measurement unit on Romi'''

    Default_addr = 0x28

    CHIP_ID       = 0x00
    PAGE_ID       = 0x07
    UNIT_SEL      = 0x3B
    OPR_MODE      = 0x3D
    PWR_MODE      = 0x3E
    SYS_TRIGGER   = 0x3F
    CALIB_STAT    = 0x35

    EULER_H_LSB   = 0x1A
    GYRO_X_LSB    = 0x14

    CALIB_START   = 0x55
    CALIB_LEN     = 22

    MODE_CONFIG       = 0x00
    MODE_IMU          = 0x08
    MODE_COMPASS      = 0x09
    MODE_M4G          = 0x0A
    MODE_NDOF_FMC_OFF = 0x0B
    MODE_NDOF         = 0x0C

    def __init__(self, i2c: pyb.I2C, addr: int = Default_addr):
        self.i2c = i2c
        self.addr = addr

        self.set_page(0)

        chip = self.read_u8(self.CHIP_ID)
        if chip != 0xA0:
            time.sleep_ms(650)
            chip = self.read_u8(self.CHIP_ID)
            if chip != 0xA0:
                raise OSError("BNO055 not found")

        self.op_mode("CONFIG")
        self.write_u8(self.SYS_TRIGGER, 0x20)
        time.sleep_ms(650)

        self.write_u8(self.PWR_MODE, 0x00)
        time.sleep_ms(10)

        self.write_u8(self.UNIT_SEL, 0x00)
        time.sleep_ms(10)

        self.write_u8(self.SYS_TRIGGER, 0x00)
        time.sleep_ms(10)

        self.op_mode("NDOF")

    def write_u8(self, reg: int, val: int):
        self.i2c.mem_write(bytearray([val & 0xFF]), self.addr, reg)

    def read_u8(self, reg: int) -> int:
        return int.from_bytes(self.i2c.mem_read(1, self.addr, reg), "little")

    def read(self, reg: int, n: int) -> bytes:
        return bytes(self.i2c.mem_read(n, self.addr, reg))

    def write(self, reg: int, data: bytes):
        self.i2c.mem_write(data, self.addr, reg)

    def set_page(self, page: int):
        self.write_u8(self.PAGE_ID, page & 0x01)
        time.sleep_ms(2)

    def to_int16(self, lsb: int, msb: int) -> int:
        value = (msb << 8) | lsb
        if value & 0x8000:
            value -= 65536
        return value

    def op_mode(self, mode="NDOF"):
        if isinstance(mode, str):
            m = mode.upper()
            if m == "CONFIG":
                mode_val = self.MODE_CONFIG
            elif m == "IMU":
                mode_val = self.MODE_IMU
            elif m == "COMPASS":
                mode_val = self.MODE_COMPASS
            elif m == "M4G":
                mode_val = self.MODE_M4G
            elif m == "NDOF_FMC_OFF":
                mode_val = self.MODE_NDOF_FMC_OFF
            elif m == "NDOF":
                mode_val = self.MODE_NDOF
            else:
                raise ValueError("Invalid mode")
        else:
            mode_val = int(mode)

        self.write_u8(self.OPR_MODE, self.MODE_CONFIG)
        time.sleep_ms(25)
        self.write_u8(self.OPR_MODE, mode_val)
        time.sleep_ms(25)

    def cal_status(self):
        status = self.read_u8(self.CALIB_STAT)
        sys = (status >> 6) & 0x03
        gyro = (status >> 4) & 0x03
        accel = (status >> 2) & 0x03
        mag = status & 0x03
        return (sys, gyro, accel, mag)

    def cal_coeff(self):
        current_mode = self.read_u8(self.OPR_MODE)
        self.op_mode("CONFIG")
        coeffs = self.read(self.CALIB_START, self.CALIB_LEN)
        self.write_u8(self.OPR_MODE, current_mode)
        time.sleep_ms(25)
        return coeffs

    def write_coeff(self, coeff_bytes: bytes):
        if len(coeff_bytes) != self.CALIB_LEN:
            raise ValueError("Calibration data must be 22 bytes")

        current_mode = self.read_u8(self.OPR_MODE)
        self.op_mode("CONFIG")
        self.write(self.CALIB_START, coeff_bytes)
        self.write_u8(self.OPR_MODE, current_mode)
        time.sleep_ms(25)

    def read_EulerAng(self):
        data = self.read(self.EULER_H_LSB, 6)
        heading = self.to_int16(data[0], data[1]) / 16.0
        pitch   = self.to_int16(data[2], data[3]) / 16.0
        roll    = self.to_int16(data[4], data[5]) / 16.0
        return (heading, pitch, roll)

    def heading(self):
        h, _, _ = self.read_EulerAng()
        return h

    def read_AngVel(self):
        data = self.read(self.GYRO_X_LSB, 6)
        gx = self.to_int16(data[0], data[1]) / 16.0
        gy = self.to_int16(data[2], data[3]) / 16.0
        gz = self.to_int16(data[4], data[5]) / 16.0
        return (gx, gy, gz)

    def yaw_rate(self):
        _, _, gz = self.read_AngVel()
        return gz