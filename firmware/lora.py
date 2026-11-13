# lora.py - REYAX RYLR998 helper (UART0, 115200 baud)
# Copy to the Pico root on BOTH the rocket and the ground station.
import time


class LoRa:
    def __init__(self, uart):
        self.u = uart
        self.rxbuf = b""
        self.ok = 0           # number of +OK replies seen
        self.err = 0          # number of +ERR replies seen

    def at(self, cmd, wait_ms=300):
        """Blocking command - use only during setup."""
        self.u.read()                       # discard anything waiting
        self.u.write(cmd + "\r\n")
        time.sleep_ms(wait_ms)
        return self.u.read()

    def configure(self, address, network=18, band=866000000,
                    param="9,7,1,12", power=22):
        """Sets the radio at every boot so both ends always match.
        BAND is not kept after power-off unless saved with ',M',
        which is why it is set here every time."""
        self.at("AT")                        # first reply may be +ERR=2: ignore
        res = {}
        for c in ("AT+ADDRESS=%d" % address,
                    "AT+NETWORKID=%d" % network,
                    "AT+BAND=%d" % band,






                    "AT+PARAMETER=%s" % param,
                    "AT+CRFOP=%d" % power):
                res[c] = self.at(c)
        ok = all((r is not None) and (b"+OK" in r) for r in res.values())
        return ok, res

    def send(self, dest, msg):
        """Non-blocking send. Payload max 240 bytes. Keep >= ~0.5 s between
        sends at SF9/125 kHz so the previous packet has finished."""
        self.u.write("AT+SEND=%d,%d,%s\r\n" % (dest, len(msg), msg))

    def poll(self):
        """Non-blocking. Returns a list of complete reply lines (bytes)."""
        lines = []
        n = self.u.any()
        if n:
            self.rxbuf += self.u.read(n)
            if len(self.rxbuf) > 1000:
                self.rxbuf = self.rxbuf[-1000:]
            while b"\n" in self.rxbuf:
                line, self.rxbuf = self.rxbuf.split(b"\n", 1)
                line = line.strip()
                if line == b"+OK":
                        self.ok += 1
                elif line.startswith(b"+ERR"):
                        self.err += 1
                if line:
                        lines.append(line)
        return lines


def parse_rcv(line):
    """b'+RCV=1,5,HELLO,-40,11' -> (1, 'HELLO', -40, 11)      or None."""
    if not line.startswith(b"+RCV="):
        return None
    try:
        s = line.decode().strip()[5:]
        addr, length, rest = s.split(",", 2)
        data, rssi, snr = rest.rsplit(",", 2)
        return int(addr), data, int(rssi), int(snr)
    except Exception:
        return None
