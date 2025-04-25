import serial
import time
import struct
import pandas as pd
import numpy as np

# Serial port and motor configuration parameters
SERIAL_PORT = '/dev/ttyACM0'   # Modify according to your actual configuration (on Ubuntu, it might be "/dev/ttyUSB0")
BAUDRATE = 115200
TIMEOUT = 1            # Timeout in seconds
MOTOR_ID = 0x01
TOLERANCE = 5000        # ±5° corresponds to 500 (unit: 0.01°)

# Open the serial port
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)

def calc_checksum(data_bytes):
    """Calculate the checksum: sum of all bytes, taking only the lowest 8 bits."""
    return sum(data_bytes) & 0xFF

def build_command(cmd_code, data_bytes):
    """
    Construct a complete RS485 command packet:
    Command frame section: header (0x3E) + command code + ID + data length + command checksum
    Data section: data content + data checksum
    """
    header = bytearray()
    header.append(0x3E)            # Header
    header.append(cmd_code)        # Command code (e.g., 0xA6)
    header.append(MOTOR_ID)        # Motor ID
    header.append(len(data_bytes)) # Data length
    header.append(calc_checksum(header))  # Checksum for the command header

    data = bytearray(data_bytes)
    data.append(calc_checksum(data))      # Checksum for the data

    return header + data

def build_command_read(cmd_code, data_bytes):
    """Construct a read command packet (without checksum for data section)."""
    header = bytearray()
    header.append(0x3E)
    header.append(cmd_code)
    header.append(MOTOR_ID)
    header.append(len(data_bytes))
    header.append(calc_checksum(header))
    return header

def send_command(packet):
    """Send the packet to the RS485 bus."""
    ser.write(packet)

def read_encoder():
    """
    Read the current motor position (unit: 0.01°).
    This is just an example; in practice, the returned data should be parsed according to the protocol.
    """
    ser.reset_input_buffer()
    read_cmd = build_command_read(0x94, [])
    ser.write(read_cmd)
    response = ser.read(10)
    if len(response) < 10:
        return None
    angle_bytes = response[5:9]
    angle = struct.unpack('<I', angle_bytes)[0]
    angle %= 36000
    return angle

def move_motor(target_position, spin_direction, speed):
    """
    Send the "single-turn position closed-loop control command 2":
      - spin_direction: 0x00 for clockwise, 0x01 for counterclockwise
      - target_position: target position in units of 0.01° (range 0~35999)
      - speed: speed value (the unit is based on actual configuration, here assumed as 0.01 dps/LSB)
    """
    data_bytes = []
    data_bytes.append(spin_direction)
    data_bytes.extend(struct.pack('<H', target_position))
    data_bytes.append(0x00)
    data_bytes.extend(struct.pack('<I', speed))

    packet = build_command(0xA6, data_bytes)
    send_command(packet)

    # Wait until the motor reaches the target position
    while True:
        current = read_encoder()
        if current is None:
            time.sleep(0.1)
            continue
        diff = abs(current - target_position)
        if diff > 18000:
            diff = 36000 - diff
        print(f"Current angle: {current/100:.2f}°, Target angle: {target_position/100:.2f}°, Difference: {diff/100:.2f}°")
        if diff <= TOLERANCE:
            break
        time.sleep(0.2)
    time.sleep(0.1)

#############################################
# New main program: Read a new data file and control the motor
#############################################
import matplotlib.pyplot as plt

def sat(x):
    return np.tanh(x) * min(abs(x), 1)

def main_new():
    current_position = 0
    
    # angle_sequence = [0, 45, 90, 180, 270, 35, 180, 0]
    df = pd.read_csv("queen 1min angle.txt")
    angle_sequence = df["angle"].tolist()
    hold_time = 0.05  
    dt = 0.1
    e_int = 0
    e_pos_prev = 0
    min_speed =  0    
    max_speed = 80000    
    log = []  

    # SMC parameters
    lambd = 5/1
    eta = 5/1
    epsilon = 25/1

    i = 0
    start_time = time.time()
    command_start_time = time.time()

    # for target_deg in angle_sequence:
    #     new_target = int(round(target_deg * 100)) % 36000
    #     current_position = read_encoder() 
    #     current_position %= 36000

    #     diff = new_target - current_position
    #     if diff > 18000:
    #         diff -= 36000
    #     elif diff < -18000:
    #         diff += 36000
    #     spin_direction = 0x00 if diff >= 0 else 0x01
            
    #     e_pos = diff
    #     e_vel = (e_pos - e_pos_prev) / dt  
    #     e_int += e_pos * dt
    #     e_pos_prev = e_pos
        
    #     # === 🔁 Sliding Mode Controller (SMC) ===
    #     s = e_vel + lambd * e_pos
    #     u = -lambd * e_pos - eta * sat(s / epsilon)

    #     speed_val = int(min(max(abs(u), min_speed), max_speed))        

    #     move_motor(new_target, spin_direction, speed_val)
    #     now = time.time() - start_time
    #     feedback = read_encoder()
    #     error_abs = abs(target_deg - feedback * 0.01)
    #     log.append([now, target_deg, feedback * 0.01, error_abs, speed_val])
    #     time.sleep(dt)
    
    while i < len(angle_sequence):
        now = time.time() - start_time
        target_deg = angle_sequence[i]
        new_target = int(round(target_deg * 100)) % 36000
    
        current_position = read_encoder() or 0
        current_position %= 36000
    
        diff = new_target - current_position
        if diff > 18000:
            diff -= 36000
        elif diff < -18000:
            diff += 36000
        spin_direction = 0x00 if diff >= 0 else 0x01
    
        e_pos = diff
        e_vel = (e_pos - e_pos_prev) / dt
        e_int += e_pos * dt
        e_pos_prev = e_pos
    
        # Sliding Mode Controller (SMC)
        s = e_vel + lambd * e_pos
        u = -lambd * e_pos - eta * sat(s / epsilon)
        speed_val = int(min(max(abs(u), min_speed), max_speed))
    
        move_motor(new_target, spin_direction, speed_val)
    
        feedback = read_encoder()
        if feedback is not None:
            error_abs = abs(target_deg - feedback * 0.01)
            log.append([now, target_deg, feedback * 0.01, error_abs, speed_val])
    
        # Step to next target every 0.1 seconds
        if time.time() - command_start_time >= dt:
            i += 1
            command_start_time = time.time()


    print("✅ Logging complete. Saving...")

    # Save as CSV
    df = pd.DataFrame(log, columns=["time_s", "commanded_deg", "feedback_deg", "error_abs_deg", "control_command"])
    df.to_csv("angle_feedback_comparison_smc.csv", index=False)

    # Plotting
    plt.figure(figsize=(10, 5))
    plt.plot(df["time_s"], df["feedback_deg"], label="SMC tracking performance", linewidth=2)
    plt.plot(df["time_s"], df["commanded_deg"], '--', label="Desired trajectory", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title("Motor Commanded vs Feedback Angle")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()
    
    plt.figure(figsize=(10, 4))
    plt.plot(df["time_s"], df["error_abs_deg"], color='red', label="Absolute Error", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Error (deg)")
    plt.title("Absolute Tracking Error Over Time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
    
    plt.figure(figsize=(10, 4))
    plt.plot(df["time_s"], df["control_command"], color='red', label="Control command", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Angular rate (deg/sec)")
    plt.title("SMC Control Effort")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


#############################################
# Choose the main function to run based on requirements:
#############################################
if __name__ == '__main__':
    # If you need to use the original data file, call main(); here we call the new main program
    main_new()
