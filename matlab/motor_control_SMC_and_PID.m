clear
close all
A = [-135.2,  -44.69;
        32,      0];

B = [1;
     0];

C = [0.8183, 0.1581];

D = 1.004;

sys_ss = ss(A, B, C, D);

%% === SMC Parameters ===
% lambda = 250;
% eta = 34;
% epsilon = 0.7;
lambda = 250;
eta = 55;
epsilon = 3;

%% === Simulation Settings ===
% dt = 0.001;
dt = 0.002;
T = 10;
t = 0:dt:T;
n = length(t);

x = [0; 0];  % Initial 2-state system
e_int = 0;
% x2_ref = 1 * ones(size(t));  % Step reference for velocity
f = 1;  % 1 Hz sinusoidal reference
% x2_ref = sin(2 * pi * f * t); 
% x2_ref = 0.5 * (t > 2) + 0.3 * sin(4 * pi * t);  % Step at t=2 + oscillation
% x2_ref = sawtooth(2 * pi * f * t, 0.5);  % Triangle wave with period 1s
% x2_ref = zeros(size(t));
% x2_ref(t < 3) = 5 * sin(2 * pi * 1 * t(t < 3));         % [-5, +5] low-freq sine
% x2_ref(t >= 3 & t < 6) = 10;                           % step to +10
% x2_ref(t >= 6) = 3 * sin(2 * pi * 3 * t(t >= 6));      % [-3, +3] fast sine
x2_ref = zeros(size(t));
x2_ref(t < 3) = 0.5 * sin(2 * pi * 1 * t(t < 3));
x2_ref(t >= 3 & t < 6) = 1;
x2_ref(t >= 6) = 0.3 * sin(2 * pi * 3 * t(t >= 6));






x_hist = zeros(n, 2);
u_hist = zeros(n, 1);
s_hist = zeros(n, 1);

%% === SMC Simulation Loop ===
for k = 1:n
    x1 = x(1);
    x2 = x(2);

    e = x2 - x2_ref(k);             % Velocity error
    % e_int = e_int + e * dt;  % Accumulate velocity error
    e_int = e_int * 0.999 + e * dt;

    s = A(2,:) * x + lambda * e;    % Sliding surface using exact dynamics

    % Control input (no estimation)
    % u = -lambda * e - A(2,:) * x - eta * sat(s / epsilon);
    u = -lambda * e - A(2,:) * x - eta * sat(s / epsilon) - 1.5 * e_int;

    
    k1 = A*x + B*u;
    k2 = A*(x + 0.5*dt*k1) + B*u;
    x = x + dt * k2;
    % dx = A * x + B * u;
    % x = x + dx * dt;

    % Logging
    x_hist(k,:) = x;
    u_hist(k) = u;
    s_hist(k) = s;
end
x_hist_smc = x_hist;
u_hist_smc = u_hist;
x2_ref_smc = x2_ref;
%% === Plot Results ===
% figure('Color','w', 'Position', [100 100 1200 800]);
% 
% tiledlayout(2, 1, 'TileSpacing', 'tight', 'Padding', 'tight');  % 3 rows, 1 column
% 
% % --- Top Plot: Velocity Tracking ---
% nexttile
% plot(t, x_hist(:,2), 'b', 'LineWidth', 2); hold on
% plot(t, x2_ref, 'k--', 'LineWidth', 2);
% ylabel('Velocity (x_2)', 'FontSize', 24);
% legend('Actual', 'Reference', 'FontSize', 24);
% title('SMC Velocity Tracking (Exact Dynamics)', 'FontSize', 20);
% grid on
% ax = gca;
% ax.FontSize = 24;
% 
% % --- Middle Plot: Control Input ---
% nexttile
% plot(t, u_hist, 'Color', [0 0.6 0.6], 'LineWidth', 2);
% ylabel('Control Input (u)', 'FontSize', 24);
% title('Control Effort', 'FontSize', 20);
% grid on
% ax = gca;
% ax.FontSize = 24;
% hold on

% --- Bottom Plot: Sliding Surface ---
% nexttile
% plot(t, s_hist, 'r', 'LineWidth', 2);
% ylabel('Sliding Surface (s)', 'FontSize', 24);
% xlabel('Time (s)', 'FontSize', 24);
% title('Sliding Surface Evolution', 'FontSize', 20);
% grid on
% ax = gca;
% ax.FontSize = 24;


%% === Saturation Function ===
function y = sat(x)
    y = tanh(x) .* min(abs(x), 1);
end

% function y = sat(x)
%     y = x / (abs(x) + 0.5);  % smooth SMC formulation
% end

%% PID
% clear

A = [-135.2,  -44.69;
        32,      0];

B = [1;
     0];

C = [0.8183, 0.1581];

D = 1.004;

sys_ss = ss(A, B, C, D);


% Trial PID values (tune as needed)
Kp = 280;
Ki = 2;
Kd = 0.0;
C_pid_manual = pid(Kp, Ki, Kd);

dt = 0.002;
T = 10;
t = 0:dt:T;
n = length(t);
x_hist = zeros(n, 2);   % Assuming [position; velocity]
u_hist = zeros(n, 1);
e_hist = zeros(n, 1);   % Optional: track error like 's_hist' in SMC

x = [0; 0];  % Initial 2-state system
e_int = 0;

f = 1;  % Frequency in Hz
% x2_ref = sin(2 * pi * f * t);  % Sinusoidal reference input
% x2_ref = 0.5 * (t > 2) + 0.3 * sin(4 * pi * t);  % Step at t=2 + oscillation
% x2_ref = sawtooth(2 * pi * f * t, 0.5);  % Triangle wave with period 1s
% x2_ref = zeros(size(t));
% x2_ref(t < 3) = 5 * sin(2 * pi * 1 * t(t < 3));         % [-5, +5] low-freq sine
% x2_ref(t >= 3 & t < 6) = 10;                           % step to +10
% x2_ref(t >= 6) = 3 * sin(2 * pi * 3 * t(t >= 6));      % [-3, +3] fast sine
x2_ref = zeros(size(t));
x2_ref(t < 3) = 0.5 * sin(2 * pi * 1 * t(t < 3));
x2_ref(t >= 3 & t < 6) = 1;
x2_ref(t >= 6) = 0.3 * sin(2 * pi * 3 * t(t >= 6));






x = [0; 0];  % Initial condition

for k = 1:n
    % Error and PID controller logic (assuming you have x2_ref)
    e = (x2_ref(k) - x(2));
    e_int = e_int + e * dt;
    if k == 1
        de = 0;
    else
        de = (e - e_hist(k-1)) / dt;
    end
    u = Kp * e + Ki * e_int + Kd * de;


    % Apply system dynamics (RK2 or Euler)
    k1 = A*x + B*u;
    k2 = A*(x + 0.5*dt*k1) + B*u;
    x = x + dt * k2;

    % Logging
    x_hist(k,:) = x';
    u_hist(k) = u;
    e_hist(k) = e;  % Similar to s_hist
end

x_hist_pid = x_hist;
u_hist_pid = u_hist;
x2_ref_pid = x2_ref;
% figure('Color','w', 'Position', [100 100 1200 800]);

% tiledlayout(2, 1, 'TileSpacing', 'tight', 'Padding', 'tight');  % 3 rows, 1 column
% 
% % --- Top Plot: Velocity Tracking ---
% nexttile
% plot(t, x_hist(:,2), 'b', 'LineWidth', 2); hold on
% plot(t, x2_ref, 'k--', 'LineWidth', 2);
% ylabel('Velocity (x_2)', 'FontSize', 24);
% legend('Actual', 'Reference', 'FontSize', 24);
% title('SMC Velocity Tracking (Exact Dynamics)', 'FontSize', 20);
% grid on
% ax = gca;
% ax.FontSize = 24;
% 
% % --- Middle Plot: Control Input ---
% nexttile
% plot(t, u_hist, 'Color', [0 0.6 0.6], 'LineWidth', 2);
% ylabel('Control Input (u)', 'FontSize', 24);
% title('Control Effort', 'FontSize', 20);
% grid on
% ax = gca;
% ax.FontSize = 24;

% --- Bottom Plot: Sliding Surface ---
% nexttile
% plot(t, e_hist, 'r', 'LineWidth', 2);
% ylabel('Sliding Surface (s)', 'FontSize', 24);
% xlabel('Time (s)', 'FontSize', 24);
% title('Sliding Surface Evolution', 'FontSize', 20);
% grid on
% ax = gca;
% ax.FontSize = 24;


figure('Color','w', 'Position', [100 100 1200 800]);
tiledlayout(2, 1, 'TileSpacing', 'tight', 'Padding', 'tight');

% === Velocity Tracking ===
nexttile
plot(t, x_hist_smc(:,2), 'b', 'LineWidth', 2); hold on
plot(t, x_hist_pid(:,2), 'Color', [0 0.6 0.6], 'LineWidth', 2);
plot(t, x2_ref_smc, 'k--', 'LineWidth', 2);
ylabel('Velocity (deg/s)', 'Interpreter', 'latex', 'FontSize', 24);

legend({'SMC', 'PID', 'Reference'}, 'Interpreter', 'latex', 'FontSize', 24);
grid on
ax = gca;
ax.FontSize = 24;
title('Velocity Tracking: SMC vs PID', 'Interpreter', 'latex', 'FontSize', 18);

% === Control Input ===
nexttile
plot(t, u_hist_smc, 'b', 'LineWidth', 2); hold on
plot(t, u_hist_pid, 'Color', [0 0.6 0.6], 'LineWidth', 2);
ylabel('Control Input (u)', 'Interpreter', 'latex', 'FontSize', 24);
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 24);
legend({'SMC', 'PID'}, 'Interpreter', 'latex', 'FontSize', 24);
grid on
ax = gca;
ax.FontSize = 24;
title('Control Effort: SMC vs PID', 'Interpreter', 'latex', 'FontSize', 18);
