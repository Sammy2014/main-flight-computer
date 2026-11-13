# mpu6050.py - MPU6050 IMU driver (I2C)
# Copy to the Pico root. Tested on the board: WHO_AM_I 0x68 (genuine) at 0x68.
import struct, time


class MPU6050:
    _ACC_CFG = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}
    _ACC_LSB = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
    _GYR_CFG = {250: 0x00, 500: 0x08, 1000: 0x10, 2000: 0x18}
    _GYR_LSB = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}

    def __init__(self, i2c, addr=0x68, accel_range=16, gyro_range=2000):
        """accel_range in g (2/4/8/16), gyro_range in deg/s (250/500/1000/2000).
        Use 16 g / 2000 dps for flight; launch acceleration exceeds 2 g."""
        self.i2c, self.addr = i2c, addr
        self.who = self.i2c.readfrom_mem(addr, 0x75, 1)[0]
        self._w(0x6B, 0x00)                       # wake up
        time.sleep_ms(100)
        self._w(0x1A, 0x03)                       # digital low-pass ~44 Hz
        self._w(0x1B, self._GYR_CFG[gyro_range])
        self._w(0x1C, self._ACC_CFG[accel_range])
        self.a_lsb = self._ACC_LSB[accel_range]
        self.g_lsb = self._GYR_LSB[gyro_range]
        self.gbias = (0.0, 0.0, 0.0)

    def _w(self, reg, v):
        self.i2c.writeto_mem(self.addr, reg, bytes([v]))

    def raw(self):
        return struct.unpack('>hhhhhhh', self.i2c.readfrom_mem(self.addr, 0x3B, 14))

    def read(self):
        """Returns (ax, ay, az in g, gx, gy, gz in deg/s, temp degC)."""
        ax, ay, az, t, gx, gy, gz = self.raw()
        a, g, b = self.a_lsb, self.g_lsb, self.gbias
        return (ax / a, ay / a, az / a,
                gx / g - b[0], gy / g - b[1], gz / g - b[2],
                t / 340.0 + 36.53)

    def calibrate_gyro(self, n=200):
        """Keep the board STILL. Measures and stores the gyro offset."""
        sx = sy = sz = 0.0
        for _ in range(n):
            r = self.raw()
            sx += r[4]; sy += r[5]; sz += r[6]
            time.sleep_ms(5)
        g = self.g_lsb
        self.gbias = (sx / n / g, sy / n / g, sz / n / g)
        return self.gbias
