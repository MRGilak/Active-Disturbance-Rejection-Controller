%% Cascaded vs Standard ADRC Comparison
clear; close all; clc;

%% System Definition (Second-order)
% G(s) = 1 / (s^2 + 10s + 16)
num = [0 0 1];
den = [1 10 16];
[A, B, C, D] = tf2ss(num, den);
n = size(A, 1);
b0 = 1;

% Simulation parameters
dt = 0.0001;
T = 2;
N = round(T/dt);
time = (dt:dt:T)';

ctrl_div = 1;  % Controller update every ctrl_div plant samples
dT = ctrl_div * dt;

% Discretize plant
Ad = expm(A * dt);
Bd = (A \ (Ad - eye(size(A)))) * B;
Cd = C;

%% Create Standard ADRC Controller
ctrl_standard = adrc.ADRC(n);
ctrl_standard.initialize('omega_c', 6, 'omega_o', 60.0, 'b0', b0, ...
                         'dT', dT, 'uMin', -inf, 'uMax', inf , ...
                         'useCascaded', false);

%% Create Cascaded ADRC Controller
ctrl_cascaded = adrc.ADRC(n);
ctrl_cascaded.initialize('omega_c', 6, 'omega_o', [28.8, 57.6], 'b0', b0, ...
                         'dT', dT, 'uMin', -inf, 'uMax', inf, ...
                         'numObservers', 2, ...
                         'estimateLayers', [2; 2]);

%% Generate Reference Signal
w = 1;
ref = w * ones(1, numel(time));

%% Simulate
% State vectors
x_std = zeros(n, 1);
x_cas = zeros(n, 1);

% Storage
y_std = zeros(N, 1);
y_cas = zeros(N, 1);
u_std = zeros(N, 1);
u_cas = zeros(N, 1);
xhat_std = zeros(N, n+1);
observer_cas = zeros(N, n+1, 2);
f_std = zeros(N, 1);
f_cas = zeros(N, 1);

for k = 1:N
    % Standard ADRC
    if mod(k-1, ctrl_div) == 0
        u_std(k) = ctrl_standard.step(ref(k), y_std(max(1, k-1)));
        u_cas(k) = ctrl_cascaded.step(ref(k), y_cas(max(1, k-1)));
    else
        u_std(k) = u_std(max(1, k-1)); % zero-order hold
        u_cas(k) = u_cas(max(1, k-1)); % zero-order hold
    end
    
    % Plant dynamics
    x_std = Ad * x_std + Bd * u_std(k);
    y_std(k) = Cd * x_std;
    y_std(k) = y_std(k) + 0.01 * randn(1);

    x_cas = Ad * x_cas + Bd * u_cas(k);
    y_cas(k) = Cd * x_cas;
    y_cas(k) = y_cas(k) + 0.01 * randn(1);
    
    % Store estimates
    xhat_std(k, :) = ctrl_standard.getEstimatedStates()';
    observer_cas(k, :, :) = ctrl_cascaded.getObserverStates();
    
    % Store disturbance estimates
    f_std(k) = ctrl_standard.getEstimatedDisturbance();
    f_cas(k) = ctrl_cascaded.getEstimatedDisturbance();
end

%% Plot Results
figure('Name', 'Cascaded vs Standard ADRC');

% Output tracking
subplot(2,2,1);
plot(time, y_std, 'b', 'LineWidth', 1.2); hold on;
plot(time, y_cas, 'r', 'LineWidth', 1.2);
plot(time, ref, 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Output');
legend('Standard', 'Cascaded', 'Reference', 'Location', 'best');
title('Output Tracking');

% Tracking error
% subplot(2,2,2);
% plot(time, ref - y_std, 'b', 'LineWidth', 1.2); hold on;
% plot(time, ref - y_cas, 'r', 'LineWidth', 1.2);
% grid on;
% xlabel('Time (s)');
% ylabel('Error');
% legend('Standard', 'Cascaded', 'Location', 'best');
% title('Tracking Error');

% Control signal
subplot(2,2,3);
plot(time, u_std, 'b', 'LineWidth', 1.2); hold on;
plot(time, u_cas, 'r', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Control');
legend('Standard', 'Cascaded', 'Location', 'best');
title('Control Input');

% Disturbance estimation
subplot(2,2,4);
plot(time, f_std, 'b', 'LineWidth', 1.2); hold on;
plot(time, f_cas, 'r', 'LineWidth', 1.2);
plot(time, squeeze(observer_cas(:,end,1)), 'r--', 'LineWidth', 1);
plot(time, squeeze(observer_cas(:,end,2)), 'g--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Disturbance');
legend('f_{std}', 'f_{cas}', 'z_{n+1}', 'm_{n+1}', 'Location', 'best');
title('Disturbance Estimation');
