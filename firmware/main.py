# ==================================================================
# MAIN FLIGHT CONTROLLER - rocket (save on the Pico as main.py)
# Needs on the Pico: bmp388.py mpu6050.py gps.py lora.py sdcard.py
# ==================================================================
from machine import Pin
fire = Pin(19, Pin.OUT, value=0)        # deployment output OFF before anything else
led = Pin(15, Pin.OUT, value=0)

from machine import I2C, UART, SPI, ADC
import time, os, math

# ------------------------- SETTINGS -------------------------------
DRY_RUN          = True     # True: never drives the nichrome (LED only). False for flight
USE_WDT          = False    # True for flight: auto-reset if the code ever hangs

LAUNCH_G             = 2.5        # |accel| above this (g) ...
LAUNCH_SAMPLES       = 5          # ... for this many loops in a row = launch
LAUNCH_ALT_M         = 20.0       # backup launch detection: this high above the pad
CONFIRM_S            = 3.0        # within this time after launch ...
CONFIRM_ALT_M        = 5.0        # ... must have climbed this much, else back to PAD

MIN_APOGEE_S         = 2.0        # never deploy earlier than this after launch
MIN_ARM_ALT_M        = 10.0       # barometric deploy only after climbing this high
APOGEE_DROP_M        = 3.0        # deploy when altitude is this far below the maximum
APOGEE_SAMPLES       = 5          # ... for this many loops in a row
BACKUP_DEPLOY_S      = 12.0       # timer backup after launch. SET FROM YOUR SIMULATION
                                # (predicted time to apogee + 1 to 2 s)
FIRE_MS              = 1500       # nichrome on-time (use your bench-test value)

LANDED_STILL_S       = 10.0       # |vertical speed| < 0.5 m/s for this long = landed
BATT_CAL             = 13.0       # (R5 + R6) / R6, fine-tuned against a multimeter
LOOP_MS              = 20         # 50 Hz main loop
TELEM_MS             = 1000       # radio packet period
LOG_FLUSH_MS         = 500        # SD write period

ROCKET_ADDR      = 1
GROUND_ADDR      = 2
RADIO_BAND       = 866000000    # India licence-free 865-867 MHz
RADIO_POWER      = 22           # dBm
# ------------------------------------------------------------------

PAD, FLIGHT, DESCENT, LANDED = 0, 1, 2, 3
NAMES = ("PAD", "FLIGHT", "DESCENT", "LANDED")
ok_bits = 0                 # 1 baro, 2 imu, 4 gps, 8 sd, 16 radio


def blink(n, ms=80):
    for _ in range(n):
        led.on(); time.sleep_ms(ms); led.off(); time.sleep_ms(ms)
blink(3)
print("Flight computer starting. DRY_RUN =", DRY_RUN)

# ------------------------- SENSORS --------------------------------
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)

bmp = None
try:
    from bmp388 import BMP388
    bmp = BMP388(i2c)
    ok_bits |= 1
except Exception as e:
    print("BMP388 FAIL:", e)

imu = None
try:
    from mpu6050 import MPU6050
    imu = MPU6050(i2c, accel_range=16, gyro_range=2000)
    ok_bits |= 2
except Exception as e:
    print("MPU6050 FAIL:", e)

adc = ADC(26)


def batt_now():
    return adc.read_u16() * 3.3 / 65535 * BATT_CAL


gps = None
try:
    from gps import GPS
    gps = GPS(UART(1, 9600, tx=Pin(4), rx=Pin(5), timeout=0))
    ok_bits |= 4
except Exception as e:
    print("GPS FAIL:", e)

# ------------------------- RADIO ----------------------------------
radio = None
try:
    from lora import LoRa
    radio = LoRa(UART(0, 115200, tx=Pin(16), rx=Pin(17), timeout=50))
    time.sleep(1)
    rok, res = radio.configure(ROCKET_ADDR, 18, RADIO_BAND, "9,7,1,12", RADIO_POWER)
    for k in res:
        print(k, "->", res[k])
    if rok:
        ok_bits |= 16
    else:
        print("RADIO CONFIG FAILED (will still try to send)")
except Exception as e:
    print("RADIO FAIL:", e)
    radio = None

# ------------------------- SD CARD --------------------------------
logf = None
try:
    import sdcard
    spi = SPI(1, baudrate=1_000_000, sck=Pin(10), mosi=Pin(11), miso=Pin(8))
    os.mount(sdcard.SDCard(spi, Pin(12)), "/sd")
    existing = [x.lower() for x in os.listdir("/sd")]
    n = 0
    while ("flt%03d.csv" % n) in existing:
        n += 1
    logname = "/sd/flt%03d.csv" % n
    logf = open(logname, "w")
    logf.write("t_ms,state,alt_m,vel_ms,max_m,press_pa,temp_c,ax,ay,az,amag,"
                "gx,gy,gz,batt_v,fix,sats,lat,lon,gps_alt,fire\n")
    logf.flush()
    ok_bits |= 8
    print("Logging to", logname)
except Exception as e:
    print("SD FAIL:", e)
    logf = None

# ------------------------- CALIBRATION ----------------------------
print("Keep the rocket STILL and vertical on the pad...")
if imu:
    print("Gyro bias: %.2f %.2f %.2f dps" % imu.calibrate_gyro(200))
p0 = 101325.0
if bmp:
    for _ in range(150):                 # ~3 s warm-up
        bmp.read(); time.sleep_ms(20)
    s = 0.0
    for _ in range(50):
        s += bmp.read()[1]; time.sleep_ms(20)
    p0 = s / 50
    print("Pad pressure: %.1f Pa" % p0)
print("Sensor status bits:", ok_bits, "(31 = everything OK)")
blink(bin(ok_bits).count("1"), 200)           # one blink per working module

wdt = None
if USE_WDT:
    from machine import WDT
    wdt = WDT(timeout=5000)

# ------------------------- FLIGHT STATE ---------------------------
state = PAD
alt_f = 0.0; prev_alt = 0.0; vel = 0.0; max_alt = 0.0
press = p0; temp = 0.0
ax = ay = 0.0; az = 1.0; amag = 1.0; gx = gy = gz = 0.0
batt = batt_now()
hi_count = 0; below = 0; still_ms = 0
t_launch = None; t_deploy = None
fired = 0                    # 0 = not fired, 1 = barometric, 2 = timer backup
fire_t = None                # time the nichrome was switched on
pkt = 0; buf = []
t_prev = time.ticks_ms(); t_tx = t_prev; t_log = t_prev; t_print = t_prev; t_vel = t_prev


def deploy(reason):
    global fired, fire_t, state, t_deploy
    if fired:
        return
    fired = reason
    fire_t = time.ticks_ms()
    t_deploy = fire_t
    if not DRY_RUN:
        fire.on()
    led.on()
    state = DESCENT
    print(">>> DEPLOY (%s) at %.1f m" % ("BARO" if reason == 1 else "TIMER", max_alt))


def t_since(t):
    return time.ticks_diff(time.ticks_ms(), t) / 1000.0 if t is not None else 0.0


while True:
    now = time.ticks_ms()
    dt = time.ticks_diff(now, t_prev) / 1000.0
    t_prev = now
    if wdt:
        wdt.feed()

    # Deployment timing remains inactive while DRY_RUN=True.
    if fire_t is not None and time.ticks_diff(now, fire_t) >= FIRE_MS:
        fire.off()
        fire_t = None

    try:
        # Read barometer
        if bmp:
            try:
                temp, press = bmp.read()
                alt = BMP388.altitude(press, p0)
                alt_f += 0.3 * (alt - alt_f)
                vdt = time.ticks_diff(now, t_vel) / 1000.0
                if vdt >= 0.25:
                    vel += 0.5 * ((alt_f - prev_alt) / vdt - vel)
                    prev_alt = alt_f
                    t_vel = now
            except Exception as e:
                print("BMP read error:", e)

        # Read IMU
        if imu:
            try:
                ax, ay, az, gx, gy, gz, _t = imu.read()
                amag = math.sqrt(ax * ax + ay * ay + az * az)
            except Exception as e:
                print("IMU read error:", e)

        if gps:
            gps.update()
        batt += 0.05 * (batt_now() - batt)
        if radio:
            radio.poll()

        # State machine; DRY_RUN prevents electrical deployment output.
        if state != PAD and alt_f > max_alt:
            max_alt = alt_f

        if state == PAD:
            hi_count = hi_count + 1 if amag > LAUNCH_G else 0
            if hi_count >= LAUNCH_SAMPLES or (bmp and alt_f > LAUNCH_ALT_M):
                state = FLIGHT
                t_launch = now
                max_alt = alt_f
                below = 0
                print(">>> LAUNCH detected")
            elif bmp and abs(vel) < 1.0:
                p0 += 0.001 * (press - p0)

        elif state == FLIGHT:
            tf = t_since(t_launch)
            if bmp and tf > CONFIRM_S and max_alt < CONFIRM_ALT_M:
                state = PAD
                t_launch = None
                hi_count = 0
                print(">>> launch NOT confirmed, back to PAD")
            else:
                if bmp and tf > MIN_APOGEE_S and max_alt > MIN_ARM_ALT_M:
                    below = below + 1 if alt_f < max_alt - APOGEE_DROP_M else 0
                    if below >= APOGEE_SAMPLES:
                        deploy(1)
                if state == FLIGHT and tf >= BACKUP_DEPLOY_S:
                    deploy(2)

        elif state == DESCENT:
            still_ms = still_ms + int(dt * 1000) if abs(vel) < 0.5 else 0
            if still_ms >= LANDED_STILL_S * 1000 and t_since(t_deploy) > 5:
                state = LANDED
                print(">>> LANDED")

        # SD logging
        if logf:
            fix = gps.fix if gps else 0
            buf.append("%d,%d,%.2f,%.2f,%.2f,%.1f,%.2f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,"
                       "%.2f,%d,%d,%.6f,%.6f,%.1f,%d\n" % (
                now, state, alt_f, vel, max_alt, press, temp,
                ax, ay, az, amag, gx, gy, gz, batt, fix,
                gps.sats if gps else 0, gps.lat if gps else 0.0,
                gps.lon if gps else 0.0, gps.alt if gps else 0.0,
                1 if fire_t is not None else 0))
            if time.ticks_diff(now, t_log) >= LOG_FLUSH_MS:
                t_log = now
                try:
                    logf.write("".join(buf))
                    logf.flush()
                except Exception as e:
                    print("SD write error:", e)
                buf = []

        # Telemetry
        if radio and time.ticks_diff(now, t_tx) >= TELEM_MS:
            t_tx = now
            msg = "%d;%d;%.1f;%.1f;%.1f;%.1f;%.2f;%d;%d;%.6f;%.6f;%d;%d" % (
                pkt, state, t_since(t_launch), alt_f, vel, max_alt, batt,
                gps.fix if gps else 0, gps.sats if gps else 0,
                gps.lat if gps else 0.0, gps.lon if gps else 0.0, fired, ok_bits)
            radio.send(GROUND_ADDR, msg)
            pkt += 1

        # Status LED and console
        if fire_t is None:
            period = (1000, 0, 200, 2000)[state]
            led.value(1 if state == FLIGHT or (period and now % period < 60) else 0)
        if time.ticks_diff(now, t_print) >= 1000:
            t_print = now
            print("%-7s alt %6.1f v %5.1f max %6.1f |a| %4.2f batt %5.2f gps %d/%d fired %d" % (
                NAMES[state], alt_f, vel, max_alt, amag, batt,
                gps.fix if gps else 0, gps.sats if gps else 0, fired))

    except Exception as e:
        print("loop error:", e)

    spare = LOOP_MS - time.ticks_diff(time.ticks_ms(), now)
    if spare > 0:
        time.sleep_ms(spare)
