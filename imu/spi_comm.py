import numpy as np
import spidev
import time


class IMUDevice:
    WHOAMI = 0x0
    PWR_MGMT = 0x06
    GYRO_MGMT = 0x07
    GYRO_XOUT_H = 0x33
    GYRO_XOUT_L = 0x34
    GYRO_YOUT_H = 0x35
    GYRO_YOUT_L = 0x36
    GYRO_ZOUT_H = 0x37
    GYRO_ZOUT_L = 0x38
    ACCEL_XOUT_H = 0x2D
    ACCEL_XOUT_L = 0x2E
    ACCEL_YOUT_H = 0x2F
    ACCEL_YOUT_L = 0x30
    ACCEL_ZOUT_H = 0x31
    ACCEL_ZOUT_L = 0x32

    def init(spi_bus: int, device_no: int, bus_speed: int)
        self.gyro_bias = np.array([0, 0, 0], dtype='float64')
        self.acc_bias = np.array([0, 0, 0], dtype='float64')
        self.dps = 250
        self.accel_range = 2*9.81  # m/s^2
        self.mag_range = 4900  # microteslas
        self.spi = spidev.SpiDev()
        self.spi.max_speed_hz = bus_speed
        self.spi.mode = 0

        self.spi.open(spi_bus, device_no)

        spi.xfer2([PWR_MGMT, 0x01])  # take IMU out of sleep mode
        spi.xfer2([GYRO_MGMT, 0x0])  # enable gyro and accelerometer

    def whoami():
        whoami_result = spi.xfer2([0x80 | WHOAMI, 0x0])[1]
        return whoami_result

    def read_gyro():
        gyro_x_high = spi.xfer2([0x80|GYRO_XOUT_H, 0x0])[1]
        gyro_x_low = spi.xfer2([0x80|GYRO_XOUT_L, 0x0])[1]
        gyro_x_raw = gyro_x_high << 8 | gyro_x_low
        gyro_x_twos = (gyro_x_raw & 0x80 > 0) ? -(1<<15) + 0x7F | gyro_x_raw: gyro_x_raw
        gyro_x = gyro_x_twos * self.dps / (1 << 15)

        gyro_y_high = spi.xfer2([0x80|GYRO_YOUT_H, 0x0])[1]
        gyro_y_low = spi.xfer2([0x80|GYRO_YOUT_L, 0x0])[1]
        gyro_y_raw = gyro_y_high << 8 | gyro_x_low
        gyro_y_twos = (gyro_y_raw & 0x80 > 0) ? -(1<<15) + 0x7F | gyro_y_raw: gyro_y_raw
        gyro_y = gyro_y_twos * self.dps / (1 << 15)

        gyro_z_high = spi.xfer2([0x80|GYRO_ZOUT_H, 0x0])[1]
        gyro_z_low = spi.xfer2([0x80|GYRO_ZOUT_L, 0x0])[1]
        gyro_z_raw = gyro_z_high << 8 | gyro_z_low
        gyro_z_twos = (gyro_z_raw & 0x80 > 0) ? -(1<<15) + 0x7F | gyro_z_raw: gyro_z_raw
        gyro_z = gyro_z_twos * self.dps / (1 << 15)

        return np.array([gyro_x, gyro_y, gyro_z]) - self.gyro_bias

    def read_acc():
        accel_x_high = spi.xfer2([0x80|ACCEL_XOUT_H, 0x0])[1]
        accel_x_low = spi.xfer2([0x80|ACCEL_XOUT_L, 0x0])[1]
        accel_x_raw = accel_x_high << 8 | accel_x_low
        accel_x_twos = (accel_x_raw & 0x80 > 0) ? -(1<<15) + 0x7F | accel_x_raw: accel_x_raw
        accel_x = accel_x_twos * self.accel_range / (1 << 15)

        accel_y_high = spi.xfer2([0x80|ACCEL_YOUT_H, 0x0])[1]
        accel_y_low = spi.xfer2([0x80|ACCEL_YOUT_L, 0x0])[1]
        accel_y_raw = accel_y_high << 8 | accel_x_low
        accel_y_twos = (accel_y_raw & 0x80 > 0) ? -(1<<15) + 0x7F | accel_y_raw: accel_y_raw
        accel_y = accel_y_twos * self.accel_range / (1 << 15)

        accel_z_high = spi.xfer2([0x80|ACCEL_ZOUT_H, 0x0])[1]
        accel_z_low = spi.xfer2([0x80|ACCEL_ZOUT_L, 0x0])[1]
        accel_z_raw = accel_range_z_high << 8 | accel_range_z_low
        accel_z_twos = (accel_range_z_raw & 0x80 > 0) ? -(1<<15) + 0x7F | accel_z_raw: accel_z_raw
        accel_z = accel_range_z_twos * self.accel_range / (1 << 15)

        return np.array([accel_x, accel_y, accel_z]) - self.acc_bias
