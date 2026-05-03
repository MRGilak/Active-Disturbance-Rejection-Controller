%% Cascaded vs Standard ADRC Comparison

clear; close all; clc;

%% System Definition (Second-order)
% G(s) = 1.6 / (s^2 + 7.3s + 2.1)
num = [0 0 1.6];
den = [1 7.3 2.1];
[A, B, C, D] = tf2ss(num, den);
n = size(A, 1);
b0 = 1.6;

% Simulation parameters
dt = 0.001;
T = 15;
N = round(T/dt);
time = (dt:dt:T)';

% Discretize plant
Ad = expm(A * dt);
Bd = (A \ (Ad - eye(size(A)))) * B;
Cd = C;

%% Create Standard ADRC Controller
ctrl_standard = adrc.ADRC(n);
ctrl_standard.initialize('Tsettle', 1.0, 'kob', 10, 'b0', b0, ...
                         'dT', 0.01, 'uMin', -30, 'uMax', 30, ...
                         'Ke', 1.0, 'useCascaded', false);

%% Create Cascaded ADRC Controller
ctrl_cascaded = adrc.ADRC(n);
ctrl_cascaded.initialize('Tsettle', 1.0, 'kob', 10, 'b0', b0, ...
                         'dT', 0.01, 'uMin', -30, 'uMax', 30, ...
                         'Ke', 1.0, 'useCascaded', true);

%% Generate Reference Signal
w = 2*pi*0.2;
ref = sin(w * time);

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
zhat_cas = zeros(N, n+1);
mhat_cas = zeros(N, n+1);
f_std = zeros(N, 1);
f_cas = zeros(N, 1);

% Control update rate (10x slower than simulation)
ctrl_div = 10;

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
    
    x_cas = Ad * x_cas + Bd * u_cas(k);
    y_cas(k) = Cd * x_cas;
    
    % Store estimates
    xhat_std(k, :) = ctrl_standard.getEstimatedStates()';
    zhat_cas(k, :) = ctrl_cascaded.getZStates()';
    mhat_cas(k, :) = ctrl_cascaded.getMStates()';
    
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
subplot(2,2,2);
plot(time, ref - y_std, 'b', 'LineWidth', 1.2); hold on;
plot(time, ref - y_cas, 'r', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Error');
legend('Standard', 'Cascaded', 'Location', 'best');
title('Tracking Error');

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
plot(time, zhat_cas(:,end), 'r--', 'LineWidth', 1);
plot(time, mhat_cas(:,end), 'g--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Disturbance');
legend('f_{std}', 'f_{cas}', 'z_{n+1}', 'm_{n+1}', 'Location', 'best');
title('Disturbance Estimation');
