clc
clear

% --- Configuration ---
SERIAL_PORT = '/dev/ttyACM1';
BAUDRATE = 115200;
TIMEOUT = 1;
MOTOR_ID = 1;

last_angle_deg = [];
last_time_s = [];
angle_log = [];
velocity_log = [];
time_log = [];
initial_angle_deg = [];  % to hold the starting absolute angle

T_total = 5;        % Total run time in seconds
interval = 0.002;    % Sample every 50ms

% Open Serial Port
ser = serialport(SERIAL_PORT, BAUDRATE, 'Timeout', TIMEOUT);

% Send velocity command
speed_val = 100000;
send_velocity_command(speed_val, ser, MOTOR_ID);

% Start timing
t0 = tic;

while toc(t0) < T_total
    encoder_cmd = build_command_read(hex2dec('94'), MOTOR_ID);
    write(ser, encoder_cmd, 'uint8');
    pause(0.002);

    if ser.NumBytesAvailable >= 10
        response = read(ser, 10, 'uint8');
        angle_bytes = response(6:9);
        angle_val = typecast(uint8(angle_bytes), 'uint32');
        angle_deg_raw = double(angle_val) / 100;  % No mod → gives cumulative degree
        if isempty(initial_angle_deg)
            initial_angle_deg = angle_deg_raw;
        end

% Relative angle starting from zero
        angle_deg = angle_deg_raw - initial_angle_deg;
        current_time_s = toc(t0);

        if ~isempty(last_angle_deg) && ~isempty(last_time_s)
            dt = current_time_s - last_time_s;
            dtheta = angle_deg - last_angle_deg;

            % Handle wrap-around
            if dtheta > 180
                dtheta = dtheta - 360;
            elseif dtheta < -180
                dtheta = dtheta + 360;
            end

            velocity_deg_s = dtheta / dt;
        else
            velocity_deg_s = 0;
        end

        % Log data
        angle_log(end+1) = angle_deg;
        velocity_log(end+1) = velocity_deg_s;
        time_log(end+1) = current_time_s;

        % Update last values
        last_angle_deg = angle_deg;
        last_time_s = current_time_s;

        fprintf('[%.3fs] angle: %.2f deg | velocity: %.2f deg/s\n', current_time_s, angle_deg, velocity_deg_s);
    else
        disp('No encoder response received.');
    end

    pause(interval);
end

% Stop motor
send_velocity_command(0, ser, MOTOR_ID);

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

