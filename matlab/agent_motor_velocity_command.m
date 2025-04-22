clc
clear

% --- Configuration ---
SERIAL_PORT = '/dev/ttyACM2';
BAUDRATE = 115200;
TIMEOUT = 1;
MOTOR_ID = 1;

% Open Serial Port
ser = serialport(SERIAL_PORT, BAUDRATE, 'Timeout', TIMEOUT);

%% Send a velocity command
speed_val = 100000;  % Example: 10 deg/s
send_velocity_command(speed_val, ser, MOTOR_ID);

%% Wait for 10 seconds
pause(10);

%% Stop the motor
send_velocity_command(0, ser, MOTOR_ID);

%% Read encoder value
encoder_cmd = build_command_read(hex2dec('94'), MOTOR_ID);
write(ser, encoder_cmd, 'uint8');
pause(0.05);  % Wait a bit for response
if ser.NumBytesAvailable >= 10
    response = read(ser, 10, 'uint8');
    angle_bytes = response(6:9);  % MATLAB is 1-based index
    angle_val = typecast(uint8(angle_bytes), 'uint32');
    angle_deg = mod(double(angle_val), 36000);
    disp(['Angle: ', num2str(angle_deg / 100), ' deg']);
else
    disp('No encoder response received.');
end

%% --- Helper Functions ---
function send_velocity_command(speed_val, ser, motor_id)
    data_bytes = typecast(int32(speed_val), 'uint8');
    cmd_code = hex2dec('A2');
    packet = build_command(cmd_code, data_bytes, motor_id);
    write(ser, packet, 'uint8');
end

function checksum = calc_checksum(byte_array)
    checksum = mod(sum(uint8(byte_array)), 256);
end

function packet = build_command(cmd_code, data_bytes, motor_id)
    header = uint8([hex2dec('3E'), cmd_code, motor_id, length(data_bytes)]);
    header(end+1) = calc_checksum(header);
    data = uint8(data_bytes);
    data(end+1) = calc_checksum(data);
    packet = [header, data];
end

function packet = build_command_read(cmd_code, motor_id)
    header = uint8([hex2dec('3E'), cmd_code, motor_id, 0]);
    header(end+1) = calc_checksum(header);
    packet = header;
end

