import serial
import time
import struct
import pandas as pd
import matplotlib.pyplot as plt
import random
import threading

# === Serial Port Configuration ===
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
TIMEOUT = 1
MOTOR_ID = 0x01

# === Open Serial Port ===
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)

# === Communication Functions ===
def calc_checksum(data_bytes):
    return sum(data_bytes) & 0xFF

def build_command(cmd_code, data_bytes):
    header = bytearray([0x3E, cmd_code, MOTOR_ID, len(data_bytes)])
    header.append(calc_checksum(header))
    data = bytearray(data_bytes)
    data.append(calc_checksum(data))
    return header + data

def build_command_read(cmd_code):
    header = bytearray([0x3E, cmd_code, MOTOR_ID, 0])
    header.append(calc_checksum(header))
    return header

def send_command(packet):
    ser.write(packet)

def read_encoder():
    ser.write(build_command_read(0x94))
    response = ser.read(10)
    if len(response) < 10:
        return None
    angle_bytes = response[5:9]
    angle = struct.unpack('<I', angle_bytes)[0]
    return angle % 36000

def read_velocity():
    read_cmd = build_command_read(0x9C)
    ser.write(read_cmd)
    response = ser.read(13)
    if len(response) < 13:
        return None

    # Slice last 4 bytes for velocity
    speed_bytes = response[9:13]
    speed_val = struct.unpack('<i', bytes(speed_bytes))[0]
    return speed_val * 0.01  # Convert from 0.01 deg/s to deg/s


def send_velocity_command(speed_val):
    data_bytes = list(struct.pack('<i', int(speed_val)))
    packet = build_command(0xA2, data_bytes)
    send_command(packet)

levels = [-90, -60, -30, 0, 30, 60, 90]
n_levels = 3
velocity_sequence = [random.choice(levels) for _ in range(n_levels)]

interval = 0.002  # 2 ms sampling
samples_per_speed = int(1 / interval)  # 1 second at each speed
log = []  # Will store [time, commanded_deg_s, angle_deg, measured_deg_s]

for speed_deg_s in velocity_sequence:
    speed_val = int(speed_deg_s * 100)  # Convert to 0.01 deg/s units
    send_velocity_command(speed_val)

    for _ in range(samples_per_speed):
        t_now = len(log) * interval
        angle = read_encoder()
        velocity = read_velocity()

        if angle is not None and velocity is not None:
            angle_deg = angle * 0.01
            print(f"[{t_now:.3f}s] speed={speed_deg_s}, angle={angle_deg}, feedback_vel={velocity}")
            log.append([t_now, speed_deg_s, angle_deg, velocity])

        time.sleep(interval)

# Stop the motor
send_velocity_command(0)
print("✅ Logging complete.")

# === Convert to DataFrame and Save ===
df = pd.DataFrame(log, columns=["time_s", "commanded_deg_s", "angle_deg", "measured_deg_s"])

# === Compute Feedback Velocity with Wraparound Correction ===
angle_diff = df["angle_deg"].diff()
angle_diff = angle_diff.apply(lambda d: d - 360 if d > 180 else (d + 360 if d < -180 else d))
time_diff = df["time_s"].diff()
df["feedback_deg_s"] = angle_diff / time_diff
df["feedback_deg_s"].fillna(0, inplace=True)

# === Save CSV ===
df.to_csv("velocity_feedback_comparison.csv", index=False)

# === Plotting ===
plt.figure(figsize=(10, 5))
plt.plot(df["time_s"], df["measured_deg_s"], label="Measured Velocity (read)", linewidth=2)
plt.plot(df["time_s"], df["feedback_deg_s"], label="Estimated Velocity (angle diff)", linewidth=2)
plt.plot(df["time_s"], df["commanded_deg_s"], '--', label="Commanded Velocity", linewidth=2)
plt.xlabel("Time (s)")
plt.ylabel("Velocity (deg/s)")
plt.title("Motor Velocity: Commanded vs Feedback")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

