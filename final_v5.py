import serial
import time
import smbus
import math
import pigpio
import signal
import sys

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45

bus = smbus.SMBus(1)
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

def convert_to_degrees(raw_value, direction):
    if not raw_value or raw_value == '':
        return None
    degrees = float(raw_value[:2])
    minutes = float(raw_value[2:])
    decimal = degrees + (minutes / 60.0)
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal

dest_lat = float(input("Enter destination latitude (e.g., 41.3851 for Barcelona): "))
dest_lon = float(input("Enter destination longitude (e.g., 2.1734 for Barcelona): "))

ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=1)

class KalmanFilter:
    def __init__(self, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        self.Q_angle = Q_angle
        self.Q_bias = Q_bias
        self.R_measure = R_measure
        self.angle = 0.0
        self.bias = 0.0
        self.P = [[0, 0], [0, 0]]

    def update(self, new_rate, dt, target_angle=0):
        rate = new_rate - self.bias
        self.angle += dt * rate

        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]

        y = target_angle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle

pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon")
    sys.exit()

MOTOR_PINS = {'motor1': 13, 'motor2': 19, 'motor3': 12, 'motor4': 18}
PWM_FREQUENCY = 400
MIN_PWM = 1000
MAX_PWM = 2000
BASE_PWM_v2 = 1650
BASE_PWM_v3 = 1500

def safe_pwm(value):
    return max(MIN_PWM, min(MAX_PWM, value))

def initialize_motors():
    for pin in MOTOR_PINS.values():
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.set_PWM_frequency(pin, PWM_FREQUENCY)
        pi.set_servo_pulsewidth(pin, 0)

def arm_motors():
    for pin in MOTOR_PINS.values():
        pi.set_servo_pulsewidth(pin, 0)
    time.sleep(1)
    for pin in MOTOR_PINS.values():
        pi.set_servo_pulsewidth(pin, MIN_PWM)
    time.sleep(3)

def emergency_stop():
    for pin in MOTOR_PINS.values():
        pi.set_servo_pulsewidth(pin, 0)

def cleanup():
    emergency_stop()
    pi.stop()

def signal_handler(sig, frame):
    print("\nStopping...")
    cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

print("Starting System...")
initialize_motors()
arm_motors()
kf_x = KalmanFilter()
kf_y = KalmanFilter()
last_time = time.time()

try:
    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        gx = bus.read_byte_data(MPU6050_ADDR, GYRO_XOUT_H)
        gy = bus.read_byte_data(MPU6050_ADDR, GYRO_YOUT_H)
        gx_low = bus.read_byte_data(MPU6050_ADDR, GYRO_XOUT_H+1)
        gy_low = bus.read_byte_data(MPU6050_ADDR, GYRO_YOUT_H+1)
        gx_val = ((gx << 8) | gx_low)
        gy_val = ((gy << 8) | gy_low)
        if gx_val > 32768: gx_val -= 65536
        if gy_val > 32768: gy_val -= 65536
        gyro_x = gx_val / 131.0
        gyro_y = gy_val / 131.0

        angle_x = kf_x.update(gyro_x, dt)
        angle_y = kf_y.update(gyro_y, dt)

        Kp = 5.0
        correction_x = int(Kp * angle_x)
        correction_y = int(Kp * angle_y)
        correction_x = max(-200, min(200, correction_x))
        correction_y = max(-200, min(200, correction_y))

        pi.set_servo_pulsewidth(MOTOR_PINS['motor1'], safe_pwm(BASE_PWM_v3 - correction_x + correction_y))
        pi.set_servo_pulsewidth(MOTOR_PINS['motor3'], safe_pwm(BASE_PWM_v2 + correction_x - correction_y))
        pi.set_servo_pulsewidth(MOTOR_PINS['motor2'], safe_pwm(BASE_PWM_v2 + correction_x + correction_y))
        pi.set_servo_pulsewidth(MOTOR_PINS['motor4'], safe_pwm(BASE_PWM_v3 - correction_x - correction_y))

        line = ser.readline().decode('ascii', errors='replace').strip()
        if line.startswith('$GNGGA') or line.startswith('$GNRMC'):
            parts = line.split(',')
            if line.startswith('$GNGGA') and len(parts) > 5:
                raw_lat = parts[2]
                lat_dir = parts[3]
                raw_lon = parts[4]
                lon_dir = parts[5]
            elif line.startswith('$GNRMC') and len(parts) > 5:
                raw_lat = parts[3]
                lat_dir = parts[4]
                raw_lon = parts[5]
                lon_dir = parts[6]
            else:
                continue

            lat = convert_to_degrees(raw_lat, lat_dir)
            lon = convert_to_degrees(raw_lon, lon_dir) + 16
            if lat and lon:
                print(f"\nCurrent Location: {lat}, {lon}")
                print(f"Destination: {dest_lat}, {dest_lon}")
                print(f"Google Maps link: https://maps.google.com/?q={lat},{lon}")
        time.sleep(0.02)

except Exception as e:
    print(f"Error: {str(e)}")
    cleanup()
