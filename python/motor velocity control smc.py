import serial
import time
import struct
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

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


def send_velocity_command(speed_val):
    """
    Send velocity command using command 0xA2.
    speed_val: signed int in 0.01 deg/s
    """
    data_bytes = list(struct.pack('<i', int(speed_val)))
    packet = build_command(0xA2, data_bytes)
    send_command(packet)
    
def sat(x):
    y = np.tanh(x) * min(abs(x), 1)
    return y



df = pd.read_csv('queen 1min angle step.txt')
angle_deg_prev = 0
time_step = 0.1
e_int = 0
# control Parameters
landa = 250/100
eta = 55/10
epsilon = 3
#--------------------
interval = 0.1    
log = []           
start_time = time.time()

i = 0

try:
    while i < len(df):
        angle = read_encoder()
        t_now = time.time() - start_time
        if angle is not None:
            angle_deg = angle * 0.01
            if angle_deg_prev is not None:
                angle_diff = angle_deg - angle_deg_prev
                if angle_diff > 180:
                    angle_diff -= 360
                elif angle_diff < -180:
                    angle_diff += 360
                velocity_fed = angle_diff / time_step
            else:
                velocity_fed = 0
        
            angle_ref = df.loc[i, 'angle']
            e_pos = angle_ref - angle_deg
            e_vel = -velocity_fed
            
            e_int = e_int * 0.999 + e_vel * time_step
            
            s = e_vel + landa * e_pos
            
            u = -landa * e_pos - 32*0 * angle_deg - eta * sat(s / epsilon) - 1.5 * e_int;
            
            if u>80:
                u=80
            elif u<-80:
                u=-80
            else:
                u = u
            
            velocity_com = int(u  * 100)  # 0.01 deg/s
            send_velocity_command(velocity_com)
            log.append([t_now, velocity_fed, velocity_com*0.01])
            

            angle_deg_prev = angle_deg
            time.sleep(time_step)
            i += 1
            
    log_df = pd.DataFrame(log, columns=['t_now', 'velocity_fed', 'velocity_com'])      
    log_df.to_csv('log_with_index.txt', index=False)
    log_df = pd.read_csv('log_with_index.txt')  # ✅ Correct



    
    plt.figure(figsize=(10, 5))
    plt.plot(log_df["t_now"], log_df["velocity_fed"], label="Feedback", linewidth=2)
    plt.plot(log_df["t_now"], log_df["velocity_com"], '--', label="Commanded", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (deg/s)")
    plt.title("Motor Velocity: Commanded vs Feedback")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


            
except KeyboardInterrupt:
    print("Danger!")
    send_velocity_command(0)
    time.sleep(0.1)
    send_velocity_command(0)
    
finally:
    send_velocity_command(0)
    print("complete!")
    time.sleep(0.1)
    send_velocity_command(0)

    
    




