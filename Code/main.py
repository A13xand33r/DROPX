import serial
import struct
import time
import math
import pigpio
import subprocess
import re
import landing_reco

pi = pigpio.pi()

# CONFIG
PORT = "/dev/ttyAMA0"
BAUD = 115200
MSP_SET_RAW_RC = 200

TARGET_ALT = 50.0
ALT_DROP = 1.0
ROLL_TRIM = 1490
PITCH_TRIM = 1470
MAX_THROTTLE = 1500

# ULTRASONIC PINS
TRIG_PIN = 23
ECHO_PIN = 18

# ULTRASONIC SERVO
US_SERVO_PIN = 19
US_SERVO_FWD_PWM = 1050
US_SERVO_OFF_PWM = 0

# OBSTACLE AVOIDANCE
OBSTACLE_CM = 300.0
YAW_CENTER = 1500
YAW_RIGHT_PWM_AVOID = 1560
YAW_LEFT_PWM_AVOID = 1440
FORWARD_SECONDS_AFTER_CLEAR = 5.0
CLEAR_COUNT_REQUIRED = 3
AVOID_FORWARD_TILT = 70

# APPROACH LOITER CONFIG
MAX_TILT = 200
APPROACH_RADIUS_M = 12.0
LOITER_RADIUS_M = 2.5
MIN_APPROACH_SCALE = 0.2
LOITER_TILT = 35
LOITER_PERIOD = 6.0
PATTERN_CHECK_INTERVAL = 2.0

# YAW TOWARD DESTINATION
YAW_KP = 2.0
YAW_MIN_PWM = 1300
YAW_MAX_PWM = 1700
HEADING_UPDATE_INTERVAL = 0.2

# GPS STATE
DRONE_LAT = 0.0
DRONE_LON = 0.0
DRONE_ALT = 0.0
DRONE_HEADING_DEG = 0.0
DEST_LAT = 0.0
DEST_LON = 0.0

# GPIO
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class DroneInterface:
    def __init__(self):
        self.ser = serial.Serial(PORT, BAUD, timeout=0.05)
        self.rc = [1000, ROLL_TRIM, PITCH_TRIM, 1500, 1000, 1500, 1500, 1500]

    def send(self, cmd, payload):
        cs = len(payload) ^ cmd
        for b in payload:
            cs ^= b
        self.ser.write(b"$M<" + bytes([len(payload), cmd]) + payload + bytes([cs]))

    def set_rc(self, throttle):
        throttle = min(throttle, MAX_THROTTLE)
        self.rc[0] = int(throttle)
        self.send(MSP_SET_RAW_RC, struct.pack("<8H", *self.rc))


# QR
def load_destination_from_qr():
    global DEST_LAT, DEST_LON
    out = subprocess.check_output(["python3", "QR_decode.py"], text=True)
    m = re.search(
        r"Coordinates:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*([+-]?\d+(?:\.\d+)?)",
        out
    )
    if not m:
        raise RuntimeError("Invalid QR coordinates format")
    DEST_LAT = float(m.group(1))
    DEST_LON = float(m.group(2))
    if DEST_LAT == 0.0 and DEST_LON == 0.0:
        raise RuntimeError("QR destination is 0.0,0.0")


# MSP helpers
def msp_request(ser, cmd_byte, resp_size, resp_cmd_byte, wait_s=0.08):
    ser.reset_input_buffer()
    ser.write(b"$M<" + bytes([0, cmd_byte, cmd_byte]))
    time.sleep(wait_s)

    if not ser.in_waiting:
        return None

    data = ser.read(ser.in_waiting)
    hdr = b"\x24\x4d\x3e" + bytes([resp_size, resp_cmd_byte])
    i = data.find(hdr)
    if i == -1:
        return None

    start = i + 5
    end = start + resp_size
    if len(data) < end:
        return None
    return data[start:end]


# GPS MSP 106
def get_gps_raw(ser):
    payload = msp_request(ser, 0x6A, 14, 0x6A, wait_s=0.10)
    if payload is None or len(payload) != 14:
        return None

    fix, sats, lat, lon, alt, speed = struct.unpack("<BBiiHH", payload)
    return {
        "lat": lat / 1e7,
        "lon": lon / 1e7,
        "alt_m": float(alt),
        "fix": fix,
        "sats": sats
    }


# ATTITUDE MSP 108
def get_heading_deg(ser):
    payload = msp_request(ser, 0x6C, 6, 0x6C, wait_s=0.06)
    if payload is None or len(payload) != 6:
        return None

    angx, angy, heading = struct.unpack("<hhh", payload)
    h = float(heading) % 360.0
    return h


def wait_for_3d_fix(drone):
    global DRONE_LAT, DRONE_LON, DRONE_ALT, DRONE_HEADING_DEG
    last_heading_t = 0.0

    while True:
        gps = get_gps_raw(drone.ser)
        if gps and gps["lat"] != 0 and gps["lon"] != 0 and gps["fix"] >= 2:
            DRONE_LAT = gps["lat"]
            DRONE_LON = gps["lon"]
            DRONE_ALT = gps["alt_m"]

            now = time.time()
            if now - last_heading_t >= HEADING_UPDATE_INTERVAL:
                h = get_heading_deg(drone.ser)
                if h is not None:
                    DRONE_HEADING_DEG = h
                last_heading_t = now
            return
        time.sleep(0.2)


def update_state(drone, last_heading_t):
    global DRONE_LAT, DRONE_LON, DRONE_ALT, DRONE_HEADING_DEG

    gps = get_gps_raw(drone.ser)
    if gps and gps["lat"] != 0 and gps["lon"] != 0:
        DRONE_LAT = gps["lat"]
        DRONE_LON = gps["lon"]
        DRONE_ALT = gps["alt_m"]

    now = time.time()
    if now - last_heading_t >= HEADING_UPDATE_INTERVAL:
        h = get_heading_deg(drone.ser)
        if h is not None:
            DRONE_HEADING_DEG = h
        last_heading_t = now

    return last_heading_t


# ULTRASONIC
def read_distance_cm():
    if not GPIO_AVAILABLE:
        return -1.0

    GPIO.output(TRIG_PIN, False)
    time.sleep(2e-6)
    GPIO.output(TRIG_PIN, True)
    time.sleep(10e-6)
    GPIO.output(TRIG_PIN, False)

    t0 = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        if time.time() - t0 > 0.02:
            return -1.0

    t1 = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        if time.time() - t1 > 0.02:
            return -1.0

    return (time.time() - t1) * 17150.0


def read_distance_stable_cm(samples=3, sample_delay=0.004):
    vals = []
    for _ in range(samples):
        d = read_distance_cm()
        if d > 0:
            vals.append(d)
        time.sleep(sample_delay)
    if not vals:
        return -1.0
    vals.sort()
    return vals[len(vals) // 2]


# NAV
def bearing_deg(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)

    y = math.sin(dl) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dl)
    br = math.degrees(math.atan2(y, x))
    return (br + 360.0) % 360.0


def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
    return R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0-a))


def wrap_deg180(x):
    return (x + 180.0) % 360.0 - 180.0


def yaw_pwm_toward_destination():
    desired = bearing_deg(DRONE_LAT, DRONE_LON, DEST_LAT, DEST_LON)
    err = wrap_deg180(desired - DRONE_HEADING_DEG)
    pwm = int(YAW_CENTER + (YAW_KP * err))
    if pwm < YAW_MIN_PWM:
        pwm = YAW_MIN_PWM
    if pwm > YAW_MAX_PWM:
        pwm = YAW_MAX_PWM
    return pwm


def main():
    drone = DroneInterface()

    wait_for_3d_fix(drone)

    pi.set_servo_pulsewidth(US_SERVO_PIN, US_SERVO_FWD_PWM)

    throttle = 1000
    target_reached_rpm = None
    rpm_1m_below = None
    stabilized = False

    last_pattern_check = 0.0
    last_heading_t = 0.0

    avoiding = False
    avoid_phase = None
    turn_right_start = None
    turn_right_duration = 0.0
    forward_start = None
    turn_left_start = None
    clear_count = 0

    while True:
        last_heading_t = update_state(drone, last_heading_t)

        alt = DRONE_ALT
        if alt <= 0:
            time.sleep(0.05)
            continue

        if not stabilized:
            if target_reached_rpm is None:
                if alt < TARGET_ALT:
                    throttle += 2
                    throttle = min(throttle, MAX_THROTTLE)
                else:
                    target_reached_rpm = throttle

            elif rpm_1m_below is None:
                if alt > (TARGET_ALT - ALT_DROP):
                    throttle -= 2
                    throttle = max(throttle, 1000)
                else:
                    rpm_1m_below = throttle
                    throttle = (target_reached_rpm + rpm_1m_below + 170) / 2
                    throttle = min(throttle, MAX_THROTTLE)
                    stabilized = True

            drone.rc[1] = ROLL_TRIM
            drone.rc[2] = PITCH_TRIM
            drone.rc[3] = YAW_CENTER
            drone.set_rc(throttle)
            time.sleep(0.02)
            continue

        dist_m = haversine(DRONE_LAT, DRONE_LON, DEST_LAT, DEST_LON)

        if dist_m > LOITER_RADIUS_M:
            obstacle_cm = read_distance_stable_cm(samples=3, sample_delay=0.003)

            if (not avoiding) and (obstacle_cm > 0) and (obstacle_cm <= OBSTACLE_CM):
                avoiding = True
                avoid_phase = "turn_right"
                turn_right_start = time.time()
                turn_right_duration = 0.0
                forward_start = None
                turn_left_start = None
                clear_count = 0

            if avoiding:
                if avoid_phase == "turn_right":
                    drone.rc[1] = ROLL_TRIM
                    drone.rc[2] = PITCH_TRIM
                    drone.rc[3] = YAW_RIGHT_PWM_AVOID

                    if (obstacle_cm > OBSTACLE_CM) or (obstacle_cm <= 0):
                        clear_count += 1
                    else:
                        clear_count = 0

                    if clear_count >= CLEAR_COUNT_REQUIRED:
                        turn_right_duration = time.time() - turn_right_start
                        avoid_phase = "forward"
                        forward_start = time.time()
                        drone.rc[3] = YAW_CENTER

                elif avoid_phase == "forward":
                    drone.rc[1] = ROLL_TRIM
                    drone.rc[2] = int(PITCH_TRIM + AVOID_FORWARD_TILT)
                    drone.rc[3] = YAW_CENTER

                    if (time.time() - forward_start) >= FORWARD_SECONDS_AFTER_CLEAR:
                        avoid_phase = "turn_left_back"
                        turn_left_start = time.time()

                elif avoid_phase == "turn_left_back":
                    drone.rc[1] = ROLL_TRIM
                    drone.rc[2] = PITCH_TRIM
                    drone.rc[3] = YAW_LEFT_PWM_AVOID

                    if (time.time() - turn_left_start) >= turn_right_duration:
                        drone.rc[3] = YAW_CENTER
                        avoiding = False
                        avoid_phase = None

                drone.set_rc(throttle)
                time.sleep(0.02)
                continue

            scale = max(MIN_APPROACH_SCALE, min(1.0, dist_m / APPROACH_RADIUS_M))
            forward_tilt = int(MAX_TILT * scale)

            drone.rc[1] = ROLL_TRIM
            drone.rc[2] = int(PITCH_TRIM + forward_tilt)
            drone.rc[3] = yaw_pwm_toward_destination()
            drone.set_rc(throttle)

        else:
            drone.rc[3] = yaw_pwm_toward_destination()

            phase = (time.time() % LOITER_PERIOD) / LOITER_PERIOD
            drone.rc[1] = int(ROLL_TRIM + math.sin(2 * math.pi * phase) * LOITER_TILT)
            drone.rc[2] = int(PITCH_TRIM + math.cos(2 * math.pi * phase) * LOITER_TILT)
            drone.set_rc(throttle)

            if time.time() - last_pattern_check >= PATTERN_CHECK_INTERVAL:
                last_pattern_check = time.time()
                if landing_reco.detect_landing_pattern():
                    break

        time.sleep(0.02)

    while True:
        throttle -= 1
        drone.rc[1] = ROLL_TRIM
        drone.rc[2] = PITCH_TRIM
        drone.rc[3] = YAW_CENTER
        drone.set_rc(throttle)
        if throttle <= 1000:
            break
        time.sleep(0.03)

    pi.set_servo_pulsewidth(US_SERVO_PIN, US_SERVO_OFF_PWM)
    pi.stop()

    subprocess.run(["python3", "servo_pusni.py"], check=True)

    if GPIO_AVAILABLE:
        GPIO.cleanup()


if __name__ == "__main__":
    subprocess.run(["python3", "servo_hvani.py"], check=True)
    time.sleep(10)
    load_destination_from_qr()
    main()
