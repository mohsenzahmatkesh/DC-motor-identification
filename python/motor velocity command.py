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
    """
    Reads motor's current speed (if supported by 0x9C command).
    Returns velocity in deg/s (from 0.01 deg/s).
    """
    read_cmd = build_command_read(0x9C, [])
    ser.write(read_cmd)
    response = ser.read(13)  # response is usually 13 bytes
    if len(response) < 13:
        return None

    # Speed is at bytes 9 to 12 (last 4 bytes), signed int32
    speed_bytes = response[9:13]
    speed_val = struct.unpack('<i', speed_bytes)[0]  # signed int
    return speed_val * 0.01  # convert from 0.01 deg/s to deg/s


def send_velocity_command(speed_val):
    """
    Send velocity command using command 0xA2.
    speed_val: signed int in 0.01 deg/s
    """
    data_bytes = list(struct.pack('<i', int(speed_val)))
    packet = build_command(0xA2, data_bytes)
    send_command(packet)

# === Velocity Test Configuration ===
velocity_sequence = [800, 800, 800, 800, 60, 30, 0, -30, -60, -90, -60, -30, 0]  # deg/s
# levels = [-90, -60, -30, 0, 30, 60, 90]
# n_levels = 30                     # Number of velocity steps
# velocity_sequence = [random.choice(levels) for _ in range(n_levels)]
hold_time = 3     # seconds at each speed
interval = 0.002    # logging interval (50 ms)
log = []            # [time, commanded_deg_s, angle_deg]

start_time = time.time()

for speed_deg_s in velocity_sequence:
    speed_val = int(speed_deg_s * 100)  # 0.01 deg/s
    #print(f"Sending velocity command: {speed_deg_s:.1f} deg/s")
    send_velocity_command(speed_val)

    hold_start = time.time()
    while time.time() - hold_start < hold_time:
        t_now = time.time() - start_time
        angle = read_encoder()
        if angle is not None:
            angle_deg = angle * 0.01
            print(f"[{t_now:.3f}s] speed={speed_deg_s}, angle={angle_deg}")
            log.append([t_now, speed_deg_s, angle_deg])
        time.sleep(interval)

# Stop motor
send_velocity_command(0)
print("✅ Logging complete.")

# === Convert and Save ===
df = pd.DataFrame(log, columns=["time_s", "commanded_deg_s", "angle_deg"])

# === Compute Feedback Velocity with Wraparound Correction ===
angle_diff = df["angle_deg"].diff()
angle_diff = angle_diff.apply(lambda d: d - 360 if d > 180 else (d + 360 if d < -180 else d))
time_diff = df["time_s"].diff()
df["feedback_deg_s"] = angle_diff / time_diff
df["feedback_deg_s"].fillna(0, inplace=True)

# === Optional Smoothing ===
#df["feedback_deg_s_filtered"] = df["feedback_deg_s"].rolling(window=3, center=True).mean()

# === Save CSV ===
df.to_csv("velocity_feedback_comparison.csv", index=False)

# === Plotting ===
plt.figure(figsize=(10, 5))
plt.plot(df["time_s"], df["feedback_deg_s"], label="Feedback", linewidth=2)
plt.plot(df["time_s"], df["commanded_deg_s"], '--', label="Commanded", linewidth=2)
plt.xlabel("Time (s)")
plt.ylabel("Velocity (deg/s)")
plt.title("Motor Velocity: Commanded vs Feedback")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

