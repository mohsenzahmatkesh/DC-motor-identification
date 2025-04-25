clc
clear
close all
% Load two CSVs
data1 = readtable('angle_feedback_comparison_pid.csv');
data2 = readtable('angle_feedback_comparison_smc.csv');

% Fix rows where error_abs_deg > 300
idx1 = data1.error_abs_deg > 350;
data1.error_abs_deg(idx1) = 360 - data1.error_abs_deg(idx1);

idx2 = data2.error_abs_deg > 350;
data2.error_abs_deg(idx2) = 360 - data2.error_abs_deg(idx2);

% Extract variables
t_pid = data1.time_s;
u_pid = data1.control_command;

t_smc = data2.time_s;
u_smc = data2.control_command;

% Plot
figure('Color','w', 'Position', [100 100 1200 800]);
tiledlayout(2, 1, 'TileSpacing', 'tight', 'Padding', 'tight');

nexttile
plot(data2.time_s, data2.error_abs_deg, 'b', 'LineWidth', 2); hold on
plot(data1.time_s, data1.error_abs_deg, 'Color', [0 0.6 0.6], 'LineWidth', 2);

ylabel('Absolute Error (deg)', 'Interpreter', 'latex', 'FontSize', 24);
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 24);
legend({'SMC', 'PID'}, 'Interpreter', 'latex', 'FontSize', 24);
grid on

ax = gca;
ax.FontSize = 24;
title('Tracking Error: SMC vs PID', 'Interpreter', 'latex', 'FontSize', 18);

nexttile
plot(t_smc, u_smc, 'b', 'LineWidth', 2); hold on
plot(t_pid, u_pid, 'Color', [0 0.6 0.6], 'LineWidth', 2);

ylabel('Control Input (u)', 'Interpreter', 'latex', 'FontSize', 24);
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 24);
legend({'SMC', 'PID'}, 'Interpreter', 'latex', 'FontSize', 24);
grid on

ax = gca;
ax.FontSize = 24;
title('Control Effort: SMC vs PID', 'Interpreter', 'latex', 'FontSize', 18);


% New figure for commanded vs feedback angles
figure('Color','w', 'Position', [100 100 1200 800]);
tiledlayout(2, 1, 'TileSpacing', 'tight', 'Padding', 'tight');

% === SMC: Top Plot ===
nexttile
plot(data2.time_s, data2.feedback_deg, 'b', 'LineWidth', 2); hold on
plot(data2.time_s, data2.commanded_deg, '--k', 'LineWidth', 2);
ylabel('Angle (deg)', 'Interpreter', 'latex', 'FontSize', 24);
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 24);
legend({'SMC Command', 'Feedback'}, 'Interpreter', 'latex', 'FontSize', 24);
grid on
ax = gca;
ax.FontSize = 24;
title('SMC: Commanded vs Feedback', 'Interpreter', 'latex', 'FontSize', 18);

% === PID: Bottom Plot ===
nexttile
plot(data1.time_s, data1.feedback_deg, 'Color', [0 0.6 0.6], 'LineWidth', 2);hold on
plot(data1.time_s, data1.commanded_deg, '--k', 'LineWidth', 2); 
ylabel('Angle (deg)', 'Interpreter', 'latex', 'FontSize', 24);
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 24);
legend({'PID Command', 'Feedback'}, 'Interpreter', 'latex', 'FontSize', 24);
grid on
ax = gca;
ax.FontSize = 24;
title('PID: Commanded vs Feedback', 'Interpreter', 'latex', 'FontSize', 18);




