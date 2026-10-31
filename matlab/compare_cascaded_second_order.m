%% Compare MATLAB ADRC with the Cascaded ADRC Simulink model
% The MATLAB controller is discrete-time. The Simulink model is continuous.
% A residual difference is therefore expected; it should shrink as dT -> 0.
clear; close all; clc;

addpath(fileparts(mfilename('fullpath')));

dts = [1e-3, 5e-4, 2.5e-4, 1.25e-4];
stopTime = 4;
Tsettle = 0.5;
kob = 10;
alpha = 2;
b0 = 1;
model = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
    'Simulink', 'Cascaded ADRC', 'second_order.slx');

% G(s) = 1 / (s^2 + 10s + 16)
[A, B, C, ~] = tf2ss([0 0 1], [1 10 16]);

signalNames = {'y', 'y_cascaded', 'yest', 'fest', 'z3p3'};
results = struct('dt', {}, 'rms', {}, 'matlabData', {}, 'simulinkData', {});

fprintf('Sample-time convergence study (Simulink - MATLAB):\n');
fprintf('%-10s', 'dT');
for i = 1:numel(signalNames)
    fprintf('%-14s', signalNames{i});
end
fprintf('\n');

for idx = 1:numel(dts)
    dt = dts(idx);
    time = (0:dt:stopTime)';

    Ad = expm(A * dt);
    Bd = A \ ((Ad - eye(size(A))) * B);

    standard = adrc.ADRC(2);
    standard.initialize('Tsettle', Tsettle, 'kob', kob, 'b0', b0, ...
        'dT', dt, 'uMin', -inf, 'uMax', inf);

    cascaded = adrc.ADRC(2);
    cascaded.initialize('omega_c', 6 / Tsettle, ...
        'omega_o', (6 / Tsettle) * [kob, kob * alpha], ...
        'b0', b0, 'numObservers', 2, 'estimateLayers', [2; 2], ...
        'dT', dt, 'uMin', -inf, 'uMax', inf);

    matlabData = simulateMATLAB(standard, cascaded, Ad, Bd, C, time);
    simulinkData = simulateSimulink(model, time);

    rmsVals = zeros(1, numel(signalNames));
    for i = 1:numel(signalNames)
        name = signalNames{i};
        delta = simulinkData.(name) - matlabData.(name);
        rmsVals(i) = sqrt(mean(delta.^2));
    end

    results(idx).dt = dt;
    results(idx).rms = rmsVals;
    results(idx).matlabData = matlabData;
    results(idx).simulinkData = simulinkData;

    fprintf('%-10.3g', dt);
    for i = 1:numel(signalNames)
        fprintf('%-14.6g', rmsVals(i));
    end
    fprintf('\n');
end

fprintf('\nObserved convergence order, log2(RMS(dT)/RMS(dT/2)):\n');
fprintf('%-18s', 'dT pair');
for i = 1:numel(signalNames)
    fprintf('%-14s', signalNames{i});
end
fprintf('\n');
for idx = 2:numel(results)
    fprintf('%-18s', sprintf('%.3g->%.3g', results(idx - 1).dt, results(idx).dt));
    for i = 1:numel(signalNames)
        order = log2(results(idx - 1).rms(i) / results(idx).rms(i));
        fprintf('%-14.3f', order);
    end
    fprintf('\n');
end

finest = results(end);
matlabData = finest.matlabData;
simulinkData = finest.simulinkData;
time = (0:finest.dt:stopTime)';

figure('Name', 'MATLAB vs Simulink ADRC comparison');
subplot(3, 1, 1);
plot(time, simulinkData.y, 'k', time, matlabData.y, 'r--', ...
    time, simulinkData.y_cascaded, 'b', time, matlabData.y_cascaded, 'c--', ...
    'LineWidth', 1);
grid on; ylabel('y');
legend('Standard Simulink', 'Standard MATLAB', ...
    'Cascaded Simulink', 'Cascaded MATLAB', 'Location', 'best');
title(sprintf('Plant output, dT = %g s', finest.dt));

subplot(3, 1, 2);
plot(time, simulinkData.yest, 'k', time, matlabData.yest, 'r--', ...
    time, simulinkData.ydotest, 'b', time, matlabData.ydotest, 'c--', ...
    'LineWidth', 1);
grid on; ylabel('state estimates');
legend('yest Simulink', 'yest MATLAB', 'ydotest Simulink', ...
    'ydotest MATLAB', 'Location', 'best');

subplot(3, 1, 3);
plot(time, simulinkData.fest, 'k', time, matlabData.fest, 'r--', ...
    time, simulinkData.z3p3, 'b', time, matlabData.z3p3, 'c--', ...
    'LineWidth', 1);
grid on; xlabel('Time (s)'); ylabel('disturbance estimates');
legend('fest Simulink', 'fest MATLAB', 'z3+m3 Simulink', ...
    'z3+m3 MATLAB', 'Location', 'best');

figure('Name', 'Sample-time convergence');
loglog([results.dt], vertcat(results.rms), 'o-', 'LineWidth', 1.2);
grid on; xlabel('dT (s)'); ylabel('RMS error');
legend(signalNames, 'Location', 'best');
set(gca, 'XDir', 'reverse');
title('Simulink - MATLAB error versus controller sample time');

function data = simulateMATLAB(standard, cascaded, Ad, Bd, C, time)
    nSamples = numel(time);
    xStandard = zeros(2, 1);
    xCascaded = zeros(2, 1);
    names = {'y', 'y_cascaded', 'yest', 'ydotest', 'fest', ...
        'z1', 'z2', 'z3', 'm1', 'm2', 'm3'};
    for i = 1:numel(names)
        data.(names{i}) = zeros(nSamples, 1);
    end

    for k = 1:nSamples
        reference = double(time(k) >= 2);
        yStandard = C * xStandard;
        yCascaded = C * xCascaded;
        uStandard = standard.step(reference, yStandard);
        uCascaded = cascaded.step(reference, yCascaded);

        xStandard = Ad * xStandard + Bd * uStandard;
        xCascaded = Ad * xCascaded + Bd * uCascaded;

        standardStates = standard.getObserverStates(1);
        cascadedStates = cascaded.getObserverStates();
        data.y(k) = yStandard;
        data.y_cascaded(k) = yCascaded;
        data.yest(k) = standardStates(1);
        data.ydotest(k) = standardStates(2);
        data.fest(k) = standardStates(3);
        data.z1(k) = cascadedStates(1, 1);
        data.z2(k) = cascadedStates(2, 1);
        data.z3(k) = cascadedStates(3, 1);
        data.m1(k) = cascadedStates(1, 2);
        data.m2(k) = cascadedStates(2, 2);
        data.m3(k) = cascadedStates(3, 2);
    end
    data.z3p3 = data.z3 + data.m3;
end

function data = simulateSimulink(model, time)
    [~, modelName] = fileparts(model);
    load_system(model);

    % The model's InitFcn defines workspace variables (including b0, Ad, Bd).
    % Clear it while simulating so it cannot overwrite this script's variables.
    oldInitFcn = get_param(modelName, 'InitFcn');
    set_param(modelName, 'InitFcn', '');
    cleanup = onCleanup(@() restoreModel(modelName, oldInitFcn));

    simOut = sim(modelName, 'StopTime', num2str(time(end)), ...
        'Solver', 'ode4', 'FixedStep', num2str(time(2) - time(1)), ...
        'ReturnWorkspaceOutputs', 'on');

    names = {'y', 'y_cascaded', 'yest', 'ydotest', 'fest', ...
        'z1', 'z2', 'z3', 'm1', 'm2', 'm3'};
    for i = 1:numel(names)
        value = simOut.get(names{i});
        data.(names{i}) = resampleTimeseries(value, time);
    end
    data.z3p3 = data.z3 + data.m3;
end

function values = resampleTimeseries(value, time)
    if isa(value, 'timeseries')
        values = interp1(value.Time(:), squeeze(value.Data), time, 'previous', 'extrap');
    elseif isnumeric(value) && size(value, 2) >= 2
        values = interp1(value(:, 1), value(:, 2), time, 'previous', 'extrap');
    else
        error('ADRC:ComparisonOutput', 'Unsupported Simulink output format: %s', class(value));
    end
    values = values(:);
end

function restoreModel(model, initFcn)
    set_param(model, 'InitFcn', initFcn);
    close_system(model, 0);
end
