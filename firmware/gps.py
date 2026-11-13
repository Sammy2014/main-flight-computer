# gps.py - non-blocking NMEA reader for the NEO-M8N (UART1, 38400 baud)
# Copy to the Pico root. Call update() every loop; it never waits.
import time


def nmea_to_deg(v, hemi):
    """ddmm.mmmmm + N/S/E/W -> signed decimal degrees."""
    v = float(v)
    d = int(v / 100)
    deg = d + (v - d * 100) / 60.0
    return -deg if hemi in ("S", "W") else deg


def _checksum_ok(line):
    try:
        body, cs = line[1:].split(b"*")
        c = 0
        for ch in body:
            c ^= ch







        return c == int(cs[:2], 16)
    except Exception:
        return False


class GPS:
    def __init__(self, uart):
        self.u = uart
        self.buf = b""
        self.fix = 0          # 0 = no fix, 1 = GPS fix, 2 = DGPS
        self.sats = 0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0        # m above mean sea level
        self.last_fix_ms = None
        self.lines = 0        # count of valid sentences (proves wiring works)

    def update(self):
        n = self.u.any()
        if not n:
            return
        self.buf += self.u.read(n)
        if len(self.buf) > 600:
            self.buf = self.buf[-600:]
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            line = line.strip()
            if line.startswith(b"$") and _checksum_ok(line):
                self.lines += 1
                if line[3:6] == b"GGA":
                    self._gga(line)

    def _gga(self, line):
        try:
                f = line.decode().split(",")
                self.fix = int(f[6] or 0)
                self.sats = int(f[7] or 0)
                if self.fix > 0 and f[2] and f[4]:
                    self.lat = nmea_to_deg(f[2], f[3])
                    self.lon = nmea_to_deg(f[4], f[5])
                    self.alt = float(f[9] or 0)
                    self.last_fix_ms = time.ticks_ms()
        except Exception:
                pass
