# bmp388.py - BMP388 barometer driver (I2C), Bosch float compensation
# Copy to the Pico root. Tested on the board: chip ID 0x50 at address 0x77.
import struct, time


class BMP388:
    def __init__(self, i2c, addr=0x77):
        self.i2c, self.addr = i2c, addr
        cid = self._r(0x00, 1)[0]
        if cid != 0x50:
            raise OSError("BMP388 not found, chip ID = %s" % hex(cid))
        self._w(0x7E, 0xB6)          # soft reset
        time.sleep_ms(10)
        self._w(0x1C, 0x03)          # OSR: pressure x8, temperature x1
        self._w(0x1D, 0x02)          # ODR: 50 Hz
        self._w(0x1F, 0x04)          # IIR filter coefficient 3
        self._w(0x1B, 0x33)          # pressure + temp enabled, normal mode
        time.sleep_ms(50)
        if self._r(0x02, 1)[0] & 0x04:
            raise OSError("BMP388 config error")
        self._load_calib()

    def _r(self, reg, n):
        return self.i2c.readfrom_mem(self.addr, reg, n)

    def _w(self, reg, v):
        self.i2c.writeto_mem(self.addr, reg, bytes([v]))

    def _load_calib(self):
        (t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11) = \
            struct.unpack('<HHbhhbbHHbbhbb', self._r(0x31, 21))
        self.t1 = t1 * 256.0
        self.t2 = t2 / 1073741824.0             # 2^30
        self.t3 = t3 / 281474976710656.0        # 2^48
        self.p1 = (p1 - 16384) / 1048576.0      # 2^20
        self.p2 = (p2 - 16384) / 536870912.0    # 2^29
        self.p3 = p3 / 4294967296.0             # 2^32
        self.p4 = p4 / 137438953472.0           # 2^37
        self.p5 = p5 * 8.0
        self.p6 = p6 / 64.0
        self.p7 = p7 / 256.0
        self.p8 = p8 / 32768.0
        self.p9 = p9 / 281474976710656.0        # 2^48
        self.p10 = p10 / 281474976710656.0      # 2^48
        self.p11 = p11 / 36893488147419103232.0 # 2^65

    def read(self):
        """Returns (temperature degC, pressure Pa)."""
        d = self._r(0x04, 6)
        up = d[0] | (d[1] << 8) | (d[2] << 16)
        ut = d[3] | (d[4] << 8) | (d[5] << 16)
        x = ut - self.t1
        t = x * self.t2 + x * x * self.t3
        o1 = self.p5 + self.p6 * t + self.p7 * t * t + self.p8 * t * t * t
        o2 = up * (self.p1 + self.p2 * t + self.p3 * t * t + self.p4 * t * t * t)
        o3 = up * up * (self.p9 + self.p10 * t) + up * up * up * self.p11
        return t, o1 + o2 + o3

    @staticmethod
    def altitude(p, ref=101325.0):
        """Altitude (m) of pressure p relative to reference pressure ref."""
        return 44330.0 * (1.0 - (p / ref) ** 0.1903)
