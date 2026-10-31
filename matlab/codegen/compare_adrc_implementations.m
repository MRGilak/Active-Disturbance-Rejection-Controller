function results = compare_adrc_implementations()
%COMPARE_ADRC_IMPLEMENTATIONS Compare MATLAB ADRC with the generated MEX wrapper.

thisDir = fileparts(mfilename('fullpath'));
addpath(fileparts(thisDir));
generatedDir = fullfile(thisDir, 'generated');
if ~exist(fullfile(generatedDir, ['adrc_codegen_mex.' mexext]), 'file')
    error('ADRC:Comparison', ...
        'Generated MEX is missing. Run build_adrc_mex first.');
end
addpath(generatedDir);

order = 2;
dT = 0.001;
numSteps = 200000;
disturbance = 0.2;
reference = ones(numSteps, 1);
reference(1:20) = 0;

options = {'Tsettle', 1, 'kob', 10, 'b0', 1, 'dT', dT, ...
    'uMin', -10, 'uMax', 10};
matlabController = adrc.ADRC(order);
mexController = ADRCMex(order);
matlabController.initialize(options{:});
mexController.initialize(options{:});

matlabOutput = zeros(numSteps, 1);
mexOutput = zeros(numSteps, 1);
matlabControl = zeros(numSteps, 1);
mexControl = zeros(numSteps, 1);
matlabVelocity = 0;
mexVelocity = 0;
matlabTime = tic;

for k = 1:numSteps
    matlabControl(k) = matlabController.step(reference(k), matlabOutput(max(k - 1, 1)));

    matlabVelocity = matlabVelocity + ...
        (matlabControl(k) + disturbance) * dT;
    matlabOutput(k) = matlabOutput(max(k - 1, 1)) + matlabVelocity * dT;
end
matlabTime = toc(matlabTime);

mexTime = tic;
for k = 1:numSteps
    mexControl(k) = mexController.step(reference(k), mexOutput(max(k - 1, 1)));
    mexVelocity = mexVelocity + (mexControl(k) + disturbance) * dT;
    mexOutput(k) = mexOutput(max(k - 1, 1)) + mexVelocity * dT;
end
mexTime = toc(mexTime);

controlError = matlabControl - mexControl;
outputError = matlabOutput - mexOutput;
speedup = matlabTime / mexTime;
results = struct( ...
    'maxControlError', max(abs(controlError)), ...
    'rmsControlError', sqrt(mean(controlError.^2)), ...
    'maxOutputError', max(abs(outputError)), ...
    'finalMatlabOutput', matlabOutput(end), ...
    'finalMexOutput', mexOutput(end), ...
    'matlabTime', matlabTime, ...
    'mexTime', mexTime, ...
    'speedup', speedup);

fprintf('Maximum control error: %.3e\n', results.maxControlError);
fprintf('RMS control error:     %.3e\n', results.rmsControlError);
fprintf('Maximum output error:  %.3e\n', results.maxOutputError);
fprintf('Final MATLAB output:   %.6f\n', results.finalMatlabOutput);
fprintf('Final MEX output:      %.6f\n', results.finalMexOutput);
fprintf('MATLAB controller time: %.6f s\n', results.matlabTime);
fprintf('MEX controller time:    %.6f s\n', results.mexTime);
fprintf('MEX speedup:            %.2fx\n', results.speedup);

figure('Name', 'MATLAB ADRC vs generated MEX');
subplot(2, 1, 1);
plot(matlabOutput, 'b', 'LineWidth', 1.2);
hold on;
plot(mexOutput, 'r--', 'LineWidth', 1.2);
plot(reference, 'k:', 'LineWidth', 1);
grid on;
xlabel('Controller step');
ylabel('Output');
legend('MATLAB', 'Generated MEX', 'Reference', 'Location', 'best');

subplot(2, 1, 2);
plot(matlabControl, 'b', 'LineWidth', 1.2);
hold on;
plot(mexControl, 'r--', 'LineWidth', 1.2);
grid on;
xlabel('Controller step');
ylabel('Control input');
legend('MATLAB', 'Generated MEX', 'Location', 'best');
end
