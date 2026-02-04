%% ADRC Usage Examples

clear; close all; clc;

%% Example 1: Basic ADRC without TD
fprintf('Example 1: Basic ADRC without TD\n');

% System: G(s) = 1.6 / (s^2 + 7.3s + 2.1)
num = [0 0 1.6];
den = [1 7.3 2.1];
[A, B, C, D] = tf2ss(num, den);
n = size(A, 1);

% Simulation parameters
dt = 0.001;
T = 10;
N = round(T/dt);
time = (dt:dt:T)';

% Discretize plant
Ad = expm(A * dt);
Bd = (A \ (Ad - eye(size(A)))) * B;
Cd = C;

% Create and initialize ADRC controller
controller = ADRC(n);
controller.initialize('Tsettle', 1.0, 'kob', 10, 'b0', 1.6, ...
                     'dT', 0.01, 'uMin', -10, 'uMax', 10);

% Generate step reference
ref = zeros(N, 1);
ref(100:end) = 1;

% Simulate
x = zeros(n, 1);
y = zeros(N, 1);
u = zeros(N, 1);
xhat = zeros(N, n+1);

for k = 1:N
    % Controller update (at slower rate)
    if mod(k-1, 10) == 0
        u(k) = controller.step(ref(k), y(max(1, k-1)));
    else
        u(k) = u(k-1);
    end
    
    % Plant simulation
    x = Ad * x + Bd * u(k);
    y(k) = Cd * x;
    
    % Store estimates
    xhat(k, :) = controller.getEstimatedStates()';
end

% Plot results
figure('Name', 'Example 1: Basic ADRC');
subplot(2,1,1);
plot(time, y, 'b', 'LineWidth', 1.5); hold on;
plot(time, ref, 'r--', 'LineWidth', 1.2);
plot(time, xhat(:, 1), 'g', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Output');
legend('Output y', 'Reference r', 'Estimate $\hat{y}$', 'interpreter', 'latex', 'Location', 'best');
title('System Response');

subplot(2,1,2);
plot(time, u, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Control Input');
title('Control Signal');

%% Example 2: ADRC with Tracking Differentiator
fprintf('\nExample 2: ADRC with TD for sinusoidal reference\n');

% Reset controller with TD
controller2 = ADRC(n);
controller2.initialize('Tsettle', 1.0, 'kob', 10, 'b0', 1.6, ...
                      'dT', 0.01, 'uMin', -30, 'uMax', 30, ...
                      'TD_method', 'euler', 'TD_params', {0.6});

% Generate sinusoidal reference
w = 2*pi*0.5;
ref_sin = sin(w * time);

% Simulate
x2 = zeros(n, 1);
y2 = zeros(N, 1);
u2 = zeros(N, 1);
xhat2 = zeros(N, n+1);

for k = 1:N
    % Controller update
    if mod(k-1, 10) == 0
        u2(k) = controller2.step(ref_sin(k), y2(max(1, k-1)));
    else
        u2(k) = u2(k-1);
    end
    
    % Plant simulation
    x2 = Ad * x2 + Bd * u2(k);
    y2(k) = Cd * x2;
    
    % Store estimates
    xhat2(k, :) = controller2.getEstimatedStates()';
end

% Plot results
figure('Name', 'Example 2: ADRC with TD');
subplot(2,1,1);
plot(time, y2, 'b', 'LineWidth', 1.5); hold on;
plot(time, ref_sin, 'r--', 'LineWidth', 1.2);
plot(time, xhat2(:, 1), 'g', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Output');
legend('Output y', 'Reference r', 'Estimate $\hat{y}$', 'interpreter', 'latex', 'Location', 'best');
title('System Response with TD (Sinusoidal Reference)');

subplot(2,1,2);
plot(time, u2, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Control Input');
title('Control Signal');

%% Example 3: ADRC with Input Delay
fprintf('\nExample 3: ADRC with input delay compensation\n');

% Reset controller with input delay
inputDelay = 0.2; % 200ms delay
controller3 = ADRC(n);
controller3.initialize('Tsettle', 1.0, 'kob', 10, 'b0', 1.6, ...
                      'dT', 0.01, 'uMin', -10, 'uMax', 10, ...
                      'inputDelay', inputDelay);

% Generate step reference
ref_step = zeros(N, 1);
ref_step(1000:end) = 1.5;

% Setup plant delay buffer
delaySamples = round(inputDelay / dt);
uBuffer = zeros(delaySamples + 1, 1);

% Simulate
x3 = zeros(n, 1);
y3 = zeros(N, 1);
u3 = zeros(N, 1);
u_delayed = zeros(N, 1);
xhat3 = zeros(N, n+1);

for k = 1:N
    % Controller update
    if mod(k-1, 10) == 0
        u3(k) = controller3.step(ref_step(k), y3(max(1, k-1)));
    else
        u3(k) = u3(k-1);
    end
    
    % Apply delay to plant input
    u_delayed(k) = uBuffer(1);
    uBuffer(1:end-1) = uBuffer(2:end);
    uBuffer(end) = u3(k);
    
    % Plant simulation with delayed input
    x3 = Ad * x3 + Bd * u_delayed(k);
    y3(k) = Cd * x3;
    
    % Store estimates
    xhat3(k, :) = controller3.getEstimatedStates()';
end

% Plot results
figure('Name', 'Example 3: ADRC with Input Delay');
subplot(2,1,1);
plot(time, y3, 'b', 'LineWidth', 1.5); hold on;
plot(time, ref_step, 'r--', 'LineWidth', 1.2);
plot(time, xhat3(:, 1), 'g', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Output');
legend('Output y', 'Reference r', 'Estimate $\hat{y}$', 'interpreter', 'latex', 'Location', 'best');
title(sprintf('System Response with %dms Input Delay', round(inputDelay*1000)));

subplot(2,1,2);
plot(time, u3, 'b', 'LineWidth', 1.5); hold on;
plot(time, u_delayed, 'r--', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Control Input');
legend('Controller $u(t)$', 'Applied $u(t-\tau)$', 'interpreter', 'latex', 'Location', 'best');
title('Control Signal');

