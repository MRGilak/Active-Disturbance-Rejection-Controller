function [u, estimatedStates, observerStates, initialized] = ...
        adrc_codegen(action, reference, output, refDerivs, numRefDerivs, config)
%ADRC_CODEGEN Numeric, fixed-size ADRC core for MATLAB Coder.
%
% action: 0 = initialize, 1 = step, 2 = reset, 3 = query state.
% Arrays are bounded for four system states and four cascaded observers.

MAX_ORDER = 4;
MAX_OBSERVERS = 4;
MAX_DELAY_STEPS = 1000;
MAX_INSTANCES = 16;
NADRC_MAX = MAX_ORDER + 1;

persistent states
if isempty(states)
    states = repmat(localEmptyState(NADRC_MAX, MAX_OBSERVERS, MAX_DELAY_STEPS), ...
        MAX_INSTANCES, 1);
end

instanceId = config.instanceId;
if instanceId < 1 || instanceId > MAX_INSTANCES || instanceId ~= floor(instanceId)
    error('ADRC:CodegenConfig', 'Invalid controller instance ID');
end
state = states(instanceId);
u = state.uPrev;
estimatedStates = zeros(NADRC_MAX, 1);
observerStates = state.observerStates;

switch action
    case uint8(0)
        state = localInitialize(config, state, NADRC_MAX, MAX_OBSERVERS, MAX_DELAY_STEPS);
    case uint8(1)
        if ~state.initialized
            error('ADRC:NotInitialized', 'Controller must be initialized before use');
        end
        [u, state] = localStep(reference, output, refDerivs, numRefDerivs, ...
            config, state, NADRC_MAX);
    case uint8(2)
        state = localReset(config, state, NADRC_MAX, MAX_OBSERVERS, MAX_DELAY_STEPS);
    case uint8(3)
        % Query only.
    case uint8(4)
        state.tdY = 0;
        state.tdYd = 0;
        state.tdPrevYd = 0;
        state.tdV1 = 0;
        state.tdV2 = 0;
    case uint8(5)
        if config.inputDelaySteps < 0 || config.inputDelaySteps > MAX_DELAY_STEPS
            error('ADRC:CodegenConfig', 'Input delay exceeds the codegen limit');
        end
        state.uHistory = zeros(MAX_DELAY_STEPS + 1, 1);
        state.uHistory(1:config.inputDelaySteps + 1) = state.uPrev;
    case uint8(6)
        state = localComputeGains(config, state, NADRC_MAX);
    otherwise
        error('ADRC:Action', 'Unknown codegen action');
end

states(instanceId) = state;
estimatedStates = localEstimatedStates(config, state, NADRC_MAX);
observerStates = state.observerStates;
initialized = state.initialized;
u = state.uPrev;
end

function state = localEmptyState(nadrcMax, maxObservers, maxDelaySteps)
state = struct( ...
    'initialized', false, ...
    'uPrev', 0.0, ...
    'observerStates', zeros(nadrcMax, maxObservers), ...
    'Ad', zeros(nadrcMax, nadrcMax, maxObservers), ...
    'Beso', zeros(nadrcMax, maxObservers), ...
    'Ld', zeros(nadrcMax, maxObservers), ...
    'Bd', zeros(nadrcMax, 1), ...
    'K', zeros(maxObservers, 1), ...
    'uHistory', zeros(maxDelaySteps + 1, 1), ...
    'tdY', 0.0, ...
    'tdYd', 0.0, ...
    'tdPrevYd', 0.0, ...
    'tdV1', 0.0, ...
    'tdV2', 0.0);
end

function state = localInitialize(config, state, nadrcMax, maxObservers, maxDelaySteps)
if config.n < 1 || config.n > 4 || config.numObservers < 1 || config.numObservers > maxObservers
    error('ADRC:CodegenConfig', 'Order and observer count must be between 1 and 4');
end
if config.inputDelaySteps < 0 || config.inputDelaySteps > maxDelaySteps
    error('ADRC:CodegenConfig', 'Input delay exceeds the codegen limit');
end

state.initialized = true;
state.uPrev = config.uInit;
state.observerStates = zeros(nadrcMax, maxObservers);
for layer = 1:config.numObservers
    state.observerStates(1:config.n + 1, layer) = config.XhatInit(1:config.n + 1);
end
state.uHistory = zeros(maxDelaySteps + 1, 1);
state.uHistory(1:config.inputDelaySteps + 1) = config.uInit;
state.tdY = 0;
state.tdYd = 0;
state.tdPrevYd = 0;
state.tdV1 = 0;
state.tdV2 = 0;

state = localComputeGains(config, state, nadrcMax);
end

function state = localReset(config, state, nadrcMax, maxObservers, maxDelaySteps)
state.observerStates = zeros(nadrcMax, maxObservers);
for layer = 1:config.numObservers
    state.observerStates(1:config.n + 1, layer) = config.XhatInit(1:config.n + 1);
end
state.uPrev = config.uInit;
state.uHistory = zeros(maxDelaySteps + 1, 1);
state.uHistory(1:config.inputDelaySteps + 1) = config.uInit;
state.tdY = 0;
state.tdYd = 0;
state.tdPrevYd = 0;
state.tdV1 = 0;
state.tdV2 = 0;
end

function state = localComputeGains(config, state, nadrcMax)
n = config.n;
N = n + 1;
A = zeros(nadrcMax);
B = zeros(nadrcMax, 1);
C = zeros(1, nadrcMax);
for i = 1:n
    A(i, i + 1) = 1;
end
B(n) = config.b0;
C(1) = 1;
AdFull = expm(A * config.dT);

augmentedA = zeros(nadrcMax + 1);
augmentedA(1:N, 1:N) = A(1:N, 1:N);
augmentedA(n, N + 1) = config.b0;
discreteSystem = expm(augmentedA * config.dT);
state.Bd(:) = 0;
state.Bd(1:N) = discreteSystem(1:N, N + 1);

state.Ld = zeros(nadrcMax, 4);
state.Ad = zeros(nadrcMax, nadrcMax, 4);
state.Beso = zeros(nadrcMax, 4);
for layer = 1:config.numObservers
    observerGain = zeros(nadrcMax, 1);
    if config.useObserverGains
        observerGain = config.observerGains(:, layer);
    else
        z = exp(-config.omega_o(layer) * config.dT);
        observerGain = localObserverGain(AdFull, C, z, N, nadrcMax);
    end
    state.Ld(:, layer) = observerGain;
    observerAd = zeros(nadrcMax);
    observerAd(1:N, 1:N) = AdFull(1:N, 1:N) - ...
        observerGain(1:N) * C(1:N) * AdFull(1:N, 1:N);
    state.Ad(:, :, layer) = observerAd;
    observerBeso = zeros(nadrcMax, 1);
    observerBeso(1:N) = state.Bd(1:N) - ...
        observerGain(1:N) * C(1:N) * state.Bd(1:N);
    state.Beso(:, layer) = observerBeso;
end

state.K = zeros(4, 1);
if config.useControllerGains
    state.K(1:n) = config.controllerGains(1:n);
else
    for i = 1:n
        state.K(n - i + 1) = factorial(n) / ...
            (factorial(n - i) * factorial(i)) * config.omega_c^i;
    end
end
end

function [u, state] = localStep(reference, output, refDerivs, numRefDerivs, ...
        config, state, nadrcMax)
n = config.n;
N = n + 1;
refFiltered = reference;
tdDerivs = zeros(4, 1);
if config.useTD
    [state, refFiltered, tdDerivs] = localUpdateTD(reference, config, state);
end

uForESO = state.uHistory(1);
bcascade = zeros(nadrcMax, 1);
bcascade(n) = config.dT;
for layer = 1:config.numObservers
    if layer == 1
        observerInput = output;
        previousDisturbance = zeros(nadrcMax, 1);
    else
        observerInput = state.observerStates(1, layer - 1);
        previousDisturbance = bcascade * sum(state.observerStates(n + 1, 1:layer - 1));
    end
    state.observerStates(1:N, layer) = state.Ad(1:N, 1:N, layer) * ...
        state.observerStates(1:N, layer) + state.Beso(1:N, layer) * uForESO + ...
        state.Ld(1:N, layer) * observerInput + previousDisturbance(1:N);
end

rStates = zeros(4, 1);
rStates(1) = refFiltered;
derivativeCount = numRefDerivs;
if config.useTD
    derivativeCount = n - 1;
    refDerivs = tdDerivs;
end
m = min(n - 1, derivativeCount);
if m > 0
    rStates(2:1 + m) = refDerivs(1:m);
end
refNth = 0;
if derivativeCount >= n
    refNth = refDerivs(n);
end

selectedStates = zeros(4, 1);
for stateIndex = 1:n
    selectedStates(stateIndex) = state.observerStates(stateIndex, config.estimateLayers(stateIndex));
end
disturbance = sum(state.observerStates(n + 1, 1:config.numObservers));
u = (state.K(1:n)' * (rStates(1:n) - selectedStates(1:n)) + ...
    refNth - disturbance) / config.b0;
if isfinite(config.uMin)
    u = max(u, config.uMin);
end
if isfinite(config.uMax)
    u = min(u, config.uMax);
end
if config.inputDelaySteps > 0
    for i = 1:config.inputDelaySteps
        state.uHistory(i) = state.uHistory(i + 1);
    end
end
state.uHistory(config.inputDelaySteps + 1) = u;
state.uPrev = u;
end

function [state, y, derivatives] = localUpdateTD(reference, config, state)
derivatives = zeros(4, 1);
switch config.tdMethod
    case uint8(1)
        a = config.tdParams(1);
        state.tdY = a * state.tdY + (1 - a) * reference;
        state.tdYd = (state.tdY - state.tdV1) / config.dT;
        derivatives(1) = state.tdYd;
        if config.n >= 2
            derivatives(2) = (state.tdYd - state.tdPrevYd) / config.dT;
        end
        state.tdV1 = state.tdY;
        state.tdPrevYd = state.tdYd;
        y = state.tdY;
    case uint8(2)
        r = config.tdParams(1);
        dotV1 = state.tdV2;
        dotV2 = -r * sign(state.tdV1 - reference + ...
            state.tdV2 * abs(state.tdV2) / (2 * r));
        state.tdV1 = state.tdV1 + dotV1 * config.dT;
        state.tdV2 = state.tdV2 + dotV2 * config.dT;
        y = state.tdV1;
        derivatives(1) = state.tdV2;
    case uint8(3)
        lambda = config.tdParams(1);
        err = state.tdV1 - reference;
        state.tdV1 = state.tdV1 + (state.tdV2 - err / lambda) * config.dT;
        state.tdV2 = state.tdV2 - 2 * err / lambda * config.dT;
        y = state.tdV1;
        derivatives(1) = state.tdV2;
    case uint8(4)
        lambda1 = config.tdParams(1);
        lambda2 = config.tdParams(2);
        err = state.tdV1 - reference;
        state.tdV1 = state.tdV1 + (state.tdV2 - lambda1 * sqrt(abs(err)) * sign(err)) * config.dT;
        state.tdV2 = state.tdV2 - lambda2 * sign(err) * config.dT;
        y = state.tdV1;
        derivatives(1) = state.tdV2;
    otherwise
        alpha = config.tdParams(1);
        beta = config.tdParams(2);
        gamma = config.tdParams(3);
        r = config.tdParams(4);
        dotV1 = state.tdV2;
        dotV2 = -r^2 * tanh((beta * state.tdV1 - (1 - alpha) * reference) / gamma) - ...
            r * state.tdV2;
        state.tdV1 = state.tdV1 + dotV1 * config.dT;
        state.tdV2 = state.tdV2 + dotV2 * config.dT;
        y = state.tdV1;
        derivatives(1) = state.tdV2;
end
end

function estimatedStates = localEstimatedStates(config, state, nadrcMax)
estimatedStates = zeros(nadrcMax, 1);
for stateIndex = 1:config.n
    estimatedStates(stateIndex) = state.observerStates(stateIndex, ...
        config.estimateLayers(stateIndex));
end
estimatedStates(config.n + 1) = sum(state.observerStates(config.n + 1, ...
    1:config.numObservers));
end

function observerGain = localObserverGain(AdFull, C, z, N, nadrcMax)
observerGain = zeros(nadrcMax, 1);
switch N
    case 2
        Adual = AdFull(1:2, 1:2)';
        Bdual = (C(1:2) * AdFull(1:2, 1:2))';
        controllability = [Bdual, Adual * Bdual];
        polynomialMatrix = Adual^2 - 2 * z * Adual + z^2 * eye(2);
        observerGain(1:2) = polynomialMatrix' * (controllability' \ [0; 1]);
    case 3
        Adual = AdFull(1:3, 1:3)';
        Bdual = (C(1:3) * AdFull(1:3, 1:3))';
        controllability = [Bdual, Adual * Bdual, Adual^2 * Bdual];
        polynomialMatrix = Adual^3 - 3 * z * Adual^2 + 3 * z^2 * Adual - z^3 * eye(3);
        observerGain(1:3) = polynomialMatrix' * (controllability' \ [0; 0; 1]);
    case 4
        Adual = AdFull(1:4, 1:4)';
        Bdual = (C(1:4) * AdFull(1:4, 1:4))';
        controllability = [Bdual, Adual * Bdual, Adual^2 * Bdual, Adual^3 * Bdual];
        polynomialMatrix = Adual^4 - 4 * z * Adual^3 + 6 * z^2 * Adual^2 - ...
            4 * z^3 * Adual + z^4 * eye(4);
        observerGain(1:4) = polynomialMatrix' * (controllability' \ [0; 0; 0; 1]);
    otherwise
        Adual = AdFull';
        Bdual = (C * AdFull)';
        controllability = [Bdual, Adual * Bdual, Adual^2 * Bdual, ...
            Adual^3 * Bdual, Adual^4 * Bdual];
        polynomialMatrix = Adual^5 - 5 * z * Adual^4 + 10 * z^2 * Adual^3 - ...
            10 * z^3 * Adual^2 + 5 * z^4 * Adual - z^5 * eye(5);
        observerGain = polynomialMatrix' * (controllability' \ [0; 0; 0; 0; 1]);
end
end
