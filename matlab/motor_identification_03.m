clc
clear
close all

% === Step 1: Load the CSV Data ===
data = readtable('velocity_feedback_comparison.csv');

% Extract time, input (commanded), and output (measured)
t = data.time_s;
u = data.commanded_deg_s;
y = data.feedback_deg_s;

% === Step 2: Create iddata Object ===
Ts = mean(diff(t));  % Sampling time
sys_data = iddata(y, u, Ts);

% === Step 3: Estimate Transfer Function (tfest) ===
model_order = 2;    % number of poles
model_zeros = 2;    % number of zeros
sys_tf = tfest(sys_data, model_order, model_zeros);

% === Step 4: Convert to State-Space ===
sys_ss = ss(sys_tf)  % Convert transfer function to state-space

t_uniform = t(1):Ts:t(end);  % uniformly spaced time

% Interpolate u and y to match this time base
u_uniform = interp1(t, u, t_uniform);
y_uniform = interp1(t, y, t_uniform);  % not required for lsim, but useful for plotting

compare(sys_data, sys_tf);
% === Step 5: Simulate Model Output Using lsim ===
[y_sim, t_sim] = lsim(sys_ss, u_uniform, t_uniform);

% === Step 6: Save Simulation Result ===
result = table(t_uniform', u_uniform', y_uniform', y_sim, ...
    'VariableNames', {'time_s', 'commanded_deg_s', 'measured_deg_s', 'simulated_deg_s'});

writetable(result, 'sim_vs_real_velocity.csv');
disp("✅ Simulated output saved to 'sim_vs_real_velocity.csv'");

% === Step 7: Plot with Custom Styling ===
figure('Color','w', 'Name', 'Custom Comparison Plot', ...
       'Units', 'pixels', 'Position', [100, 100, 1200, 800]);
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight'); % Adjusts margins and spacing

nexttile
plot(t, y, 'Color', [0.5, 0, 0.5], 'LineWidth', 2.5); hold on;  % Purple → Measured
% plot(t_uniform, y_uniform, 'Color', [0.5, 0, 0.5], 'LineWidth', 2.5); hold on;

plot(t_sim, y_sim, 'Color', [0, 0.6, 0.6], 'LineWidth', 2.5);   % Teal → Simulated
xlabel('Time (s)', 'FontSize', 24, 'Interpreter', 'latex');
ylabel('Velocity (deg/s)', 'FontSize', 24, 'Interpreter', 'latex');
% title('BLDC motor model identification result', 'FontSize', 20);
legend({'BLDC motor response', 'Simulated model response'}, 'FontSize', 24, ...
       'Location', 'northeast', 'Interpreter', 'latex');
grid on;
ax = gca;
xlim([0 60]);
ax.FontSize = 24;



% %% === LQR Controller Design ===
% 
% A = sys_ss.A;
% B = sys_ss.B;
% 
% % Define LQR weighting matrices
% Q = diag([1, 1]);  % Penalize position and velocity error
% R = 1;               % Penalize control effort
% 
% % Compute optimal LQR gain
% K = lqr(A, B, Q, R);  % Standard state-feedback
% 
% %% === Reference Tracking Simulation ===
% 
% % Simulate for 10 seconds
% T_sim = 10;
% dt = Ts;  % from your data
% t_vec = 0:dt:T_sim;
% r = 90 * ones(size(t_vec));  % reference position in deg
% 
% % Initial conditions
% x = [0; 0];  % [position; velocity]
% x_log = zeros(2, length(t_vec));
% u_log = zeros(1, length(t_vec));
% 
% for k = 1:length(t_vec)
%     e = r(k) - x(1);          % position error
%     u = -K * x + e;           % outer-loop reference compensation
% 
%     dx = A * x + B * u;
%     x = x + dx * dt;
% 
%     x_log(:, k) = x;
%     u_log(k) = u;
% end
% 
% %% === Plot Results ===
% figure('Color','w', 'Position', [100 100 1200 600])
% 
% subplot(2,1,1)
% plot(t_vec, r, 'k--', 'LineWidth', 2); hold on
% plot(t_vec, x_log(1,:), 'b', 'LineWidth', 2.5)
% ylabel('Position (deg)', 'FontSize', 14)
% legend('Reference', 'Actual', 'FontSize', 14)
% title('LQR Position Control Response', 'FontSize', 16)
% grid on
% 
% subplot(2,1,2)
% plot(t_vec, u_log, 'Color', [0 0.6 0.6], 'LineWidth', 2.5)
% ylabel('Velocity Command (deg/s)', 'FontSize', 14)
% xlabel('Time (s)', 'FontSize', 14)
% title('Control Input (Velocity Command)', 'FontSize', 16)
% grid on

