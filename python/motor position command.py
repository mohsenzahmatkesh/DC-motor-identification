import serial
import time
import struct
import pandas as pd

# Serial port and motor configuration parameters
SERIAL_PORT = '/dev/ttyACM0'   # Modify according to your actual configuration (on Ubuntu, it might be "/dev/ttyUSB0")
BAUDRATE = 115200
TIMEOUT = 1            # Timeout in seconds
MOTOR_ID = 0x01
TOLERANCE = 500        # ±5° corresponds to 500 (unit: 0.01°)

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

def main_new():
    """
    Instead of reading from file, send a sequence of angles,
    collect feedback, compare and plot.
    """
    current_position = 0  # Current motor position (0.01°)
    
    # Define target angles (in degrees)
    angle_sequence = [0, 45, 90, 180, 270, 359, 180, 0]
    hold_time = 2  # seconds to hold each position
    speed_val = 4000  # movement speed (0.01 deg/s)
    log = []  # List to store [timestamp, target_deg, feedback_deg]

    start_time = time.time()

    for target_deg in angle_sequence:
        new_target = int(round(target_deg * 100)) % 36000
        diff = new_target - current_position
        if diff > 18000:
            diff -= 36000
        elif diff < -18000:
            diff += 36000
        spin_direction = 0x00 if diff >= 0 else 0x01

        print(f"Moving to {target_deg:.2f}°")
        move_motor(new_target, spin_direction, speed_val)
        current_position = new_target

        hold_start = time.time()
        while time.time() - hold_start < hold_time:
            feedback = read_encoder()
            now = time.time() - start_time
            if feedback is not None:
                log.append([now, target_deg, feedback * 0.01])
            time.sleep(0.05)  # Log every 50 ms

    print("✅ Logging complete. Saving...")

    # Save as CSV
    df = pd.DataFrame(log, columns=["time_s", "commanded_deg", "feedback_deg"])
    df.to_csv("angle_feedback_comparison.csv", index=False)

    # Plotting
    plt.figure(figsize=(10, 5))
    plt.plot(df["time_s"], df["feedback_deg"], label="Feedback", linewidth=2)
    plt.plot(df["time_s"], df["commanded_deg"], '--', label="Commanded", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title("Motor Commanded vs Feedback Angle")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


#############################################
# Choose the main function to run based on requirements:
#############################################
if __name__ == '__main__':
    # If you need to use the original data file, call main(); here we call the new main program
    main_new()
