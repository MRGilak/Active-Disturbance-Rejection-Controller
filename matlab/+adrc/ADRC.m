classdef ADRC < handle
    properties (Access = private)
        % System configuration
        n               double {mustBeInteger, mustBePositive}  % System order
        dT              double {mustBePositive}                 % Sample time
        
        % Gains and matrices
        Ld              double                                  % Observer gains, one column per ESO
        K               double                                  % Controller gains
        Ad              double                                  % Discrete ESO A matrix
        Bd              double                                  % Discrete ESO B matrix
        Beso            double                                  % Discrete ESO input matrix
        Cd              double                                  % Discrete ESO C matrix
        
        % ESO configuration
        b0              double                                  % Control gain estimate
        omega_c         double                                  % Controller pole bandwidth
        omega_o         double                                  % Observer pole bandwidth
        numObservers    double                                  % Number of cascaded ESOs
        
        % State variables
        observerStates  double                                  % Extended state estimates, one column per ESO
        estimateLayers  double                                  % ESO layer used for each estimated state
        uPrev           double                                  % Previous control input
        
        % Saturation
        uMin            double = -inf                           % Lower saturation limit
        uMax            double = inf                            % Upper saturation limit
        
        % Input delay handling
        inputDelaySteps double {mustBeInteger} = 0              % Delay steps for ESO
        uHistory        double                                  % Control history buffer
        
        % Tracking differentiator (optional)
        useTD           logical = false                         % Enable TD
        TD_obj          adrc.TD                                 % TD object instance

        % Cascaded ADRC
        useCascaded     logical = false                         % Enable cascaded structure
    end
    
    properties (SetAccess = private)
        % Read-only status properties
        isInitialized   logical = false                         % Initialization flag
        controllerOrder double                                  % n+1 (extended state dimension)
    end
    
    methods
        function obj = ADRC(systemOrder, varargin)  
            if nargin < 1
                error('ADRC:Constructor', 'System order must be specified');
            end
            
            if ~isnumeric(systemOrder) || ~isscalar(systemOrder) || ...
                    ~isfinite(systemOrder) || systemOrder < 1 || systemOrder ~= floor(systemOrder)
                error('ADRC:Constructor', 'System order must be a positive integer');
            end
            
            obj.n = systemOrder;
            obj.controllerOrder = systemOrder + 1;
            obj.observerStates = zeros(obj.controllerOrder, 1);
            obj.uPrev = 0;
            
            % Initialize TD_obj with a dummy object 
            obj.TD_obj = adrc.TD('euler', 0.01, 0.9);
        end
        
        function initialize(obj, varargin)
            % Parse inputs
            p = inputParser;
            addParameter(p, 'omega_c', 4.0, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'omega_o', 40.0, @(x) isnumeric(x) && isvector(x) && all(x > 0));
            addParameter(p, 'Tsettle', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));
            addParameter(p, 'kob', 10, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'b0', 1.0, @(x) isnumeric(x) && isscalar(x));
            addParameter(p, 'uMin', -inf, @(x) isnumeric(x) && isscalar(x));
            addParameter(p, 'uMax', inf, @(x) isnumeric(x) && isscalar(x));
            addParameter(p, 'dT', 0.01, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'XhatInit', zeros(obj.controllerOrder, 1), @(x) isnumeric(x) && isvector(x));
            addParameter(p, 'uInit', 0, @(x) isnumeric(x) && isscalar(x));
            addParameter(p, 'inputDelay', 0, @(x) isnumeric(x) && isscalar(x) && x >= 0);
            addParameter(p, 'TD_method', 'none', @(x) ischar(x) || isstring(x));
            addParameter(p, 'TD_params', {}, @iscell);
            addParameter(p, 'useCascaded', false, @(x) islogical(x) && isscalar(x));
            addParameter(p, 'numObservers', 1, @(x) isnumeric(x) && isscalar(x) && x >= 1 && x == floor(x));
            addParameter(p, 'estimateLayers', [], @(x) isnumeric(x) && isvector(x) && all(isfinite(x)) && all(x == floor(x)));
            addParameter(p, 'observerGains', [], @(x) isnumeric(x) && ismatrix(x) && all(isfinite(x(:))));
            addParameter(p, 'controllerGains', [], @(x) isnumeric(x) && isvector(x) && all(isfinite(x)));
            
            parse(p, varargin{:});
            
            % Store parameters
            obj.omega_c = p.Results.omega_c;
            obj.omega_o = p.Results.omega_o;
            if ~isempty(p.Results.Tsettle)
                obj.omega_c = 2 * (obj.n + 1) / p.Results.Tsettle;
                obj.omega_o = obj.omega_c * p.Results.kob;
            elseif ~ismember('kob', p.UsingDefaults) && ismember('omega_o', p.UsingDefaults)
                obj.omega_o = obj.omega_c * p.Results.kob;
            end
            obj.b0 = p.Results.b0;
            obj.dT = p.Results.dT;
            obj.uMin = p.Results.uMin;
            obj.uMax = p.Results.uMax;
            obj.useCascaded = p.Results.useCascaded;
            if p.Results.numObservers > 1
                obj.useCascaded = true;
            end
            obj.numObservers = p.Results.numObservers;
            if isscalar(obj.omega_o)
                obj.omega_o = repmat(obj.omega_o, 1, obj.numObservers);
            elseif numel(obj.omega_o) == obj.numObservers
                obj.omega_o = reshape(obj.omega_o, 1, []);
            else
                error('ADRC:Initialize', ...
                    'omega_o must be scalar or contain one value per observer');
            end
            obj.estimateLayers = p.Results.estimateLayers;
            if isempty(obj.estimateLayers)
                obj.estimateLayers = repmat(p.Results.numObservers, obj.n, 1);
            else
                obj.estimateLayers = obj.estimateLayers(:);
                if numel(obj.estimateLayers) ~= obj.n || any(obj.estimateLayers < 1) || ...
                        any(obj.estimateLayers > p.Results.numObservers)
                    error('ADRC:Initialize', ...
                        'estimateLayers must contain %d values between 1 and numObservers', obj.n);
                end
            end
            
            % Initialize state
            XhatInit = p.Results.XhatInit(:);
            if length(XhatInit) ~= obj.controllerOrder
                error('ADRC:Initialize', 'XhatInit must have length %d', obj.controllerOrder);
            end
            obj.observerStates = repmat(XhatInit, 1, obj.numObservers);

            obj.uPrev = p.Results.uInit;
            
            % Configure input delay
            obj.inputDelaySteps = max(0, round(p.Results.inputDelay / obj.dT));
            obj.uHistory = repmat(p.Results.uInit, obj.inputDelaySteps + 1, 1);
            
            % Compute ESO gains and matrices
            obj.computeESOGains();
            obj.computeControllerGains();
            if ~isempty(p.Results.observerGains)
                if ~isequal(size(p.Results.observerGains), [obj.controllerOrder, obj.numObservers])
                    error('ADRC:Initialize', ...
                        'observerGains must have size [%d %d]', obj.controllerOrder, obj.numObservers);
                end
                obj.Ld = p.Results.observerGains;
                A_full = zeros(obj.controllerOrder);
                for state = 1:obj.n
                    A_full(state, state + 1) = 1;
                end
                Ad_full = expm(A_full * obj.dT);
                obj.Cd = [1, zeros(1, obj.n)];
                for layer = 1:obj.numObservers
                    obj.Ad(:, :, layer) = Ad_full - obj.Ld(:, layer) * obj.Cd * Ad_full;
                    obj.Beso(:, layer) = obj.Bd - obj.Ld(:, layer) * obj.Cd * obj.Bd;
                end
            end
            if ~isempty(p.Results.controllerGains)
                controllerGains = p.Results.controllerGains(:);
                if numel(controllerGains) ~= obj.n
                    error('ADRC:Initialize', ...
                        'controllerGains must have length %d', obj.n);
                end
                obj.K = controllerGains;
            end
            
            % Setup TD if requested
            TD_method = lower(p.Results.TD_method);
            if ~strcmp(TD_method, 'none')
                obj.useTD = true;
                obj.TD_obj = adrc.TD(TD_method, obj.dT, p.Results.TD_params{:});
                obj.TD_obj.reset(0);
            else
                obj.useTD = false;
                % Create a default TD object
                obj.TD_obj = adrc.TD('euler', obj.dT, 0.9);
            end
            
            obj.isInitialized = true;
        end
        
        function u = step(obj, reference, output, varargin)
       
            if ~obj.isInitialized
                error('ADRC:NotInitialized', 'Controller must be initialized before use');
            end
            
            % Update TD if enabled
            if obj.useTD
                obj.TD_obj.update(reference);
                ref_filtered = obj.TD_obj.y;
                refDerivs = [obj.TD_obj.yd];
                if obj.n >= 2
                    refDerivs = [refDerivs; obj.TD_obj.ydd];
                end
            else
                ref_filtered = reference;
                refDerivs = [];
            end
            
            % Parse optional reference derivatives from varargin
            if ~isempty(varargin)
                if isscalar(varargin) && isvector(varargin{1})
                    refDerivs = varargin{1}(:);
                else
                    try
                        refDerivs = cellfun(@(x) x, varargin);
                        refDerivs = refDerivs(:);
                    catch
                        refDerivs = [];
                    end
                end
            end
            
            % Update ESO (using delayed control input)
            u_for_eso = obj.uHistory(1);
            Bcascade = zeros(obj.controllerOrder, 1);
            Bcascade(obj.n) = obj.dT;
            for layer = 1:size(obj.observerStates, 2)
                if layer == 1
                    observerInput = output;
                    previousDisturbance = 0;
                else
                    observerInput = obj.observerStates(1, layer - 1);
                    previousDisturbance = Bcascade * sum(obj.observerStates(obj.n + 1, 1:layer - 1));
                end
                obj.observerStates(:, layer) = obj.Ad(:, :, layer) * obj.observerStates(:, layer) + ...
                    obj.Beso(:, layer) * u_for_eso + obj.Ld(:, layer) * observerInput + previousDisturbance;
            end

            % Build reference state vector
            r_states = zeros(obj.n, 1);
            r_states(1) = ref_filtered;
            
            if ~isempty(refDerivs)
                m = min(obj.n - 1, numel(refDerivs));
                if m > 0
                    r_states(2:1+m) = refDerivs(1:m);
                end
            end
            
            % nth derivative for feedforward
            ref_nth = 0;
            if numel(refDerivs) >= obj.n
                ref_nth = refDerivs(obj.n);
            end
            
            % Control law with disturbance rejection
            selectedStates = obj.getSelectedStates();
            u = (dot(obj.K, r_states - selectedStates) + ...
                ref_nth - obj.getEstimatedDisturbance()) / obj.b0;

            % Apply saturation
            u = obj.saturate(u);
            
            % Update control history for delay emulation
            if isscalar(obj.uHistory)
                obj.uHistory(1) = u;
            else
                obj.uHistory(1:end-1) = obj.uHistory(2:end);
                obj.uHistory(end) = u;
            end
            
            obj.uPrev = u;
        end
        
        function reset(obj, XhatInit, uInit)      
            if nargin < 2 || isempty(XhatInit)
                XhatInit = zeros(obj.controllerOrder, 1);
            end
            if nargin < 3 || isempty(uInit)
                uInit = 0;
            end
            
            obj.observerStates = repmat(XhatInit(:), 1, size(obj.observerStates, 2));
            obj.uPrev = uInit;
            obj.uHistory = repmat(uInit, obj.inputDelaySteps + 1, 1);
            
            if obj.useTD
                obj.TD_obj.reset(0);
            end
        end
        
        function setTD(obj, method, varargin)
            method = lower(method);
            if strcmp(method, 'none')
                obj.useTD = false;
                % Don't clear TD_obj, just disable it
            else
                obj.useTD = true;
                if ~isa(obj.TD_obj, 'TD')
                    obj.TD_obj = adrc.TD(method, obj.dT, varargin{:});
                else
                    % Update existing TD - need to recreate with new method
                    obj.TD_obj = adrc.TD(method, obj.dT, varargin{:});
                    obj.TD_obj.reset(0);
                end
            end
        end
        
        function setSaturation(obj, uMin, uMax)
            obj.uMin = uMin;
            obj.uMax = uMax;
        end
        
        function setInputDelay(obj, delaySec)
            newDelaySteps = max(0, round(delaySec / obj.dT));
            
            if newDelaySteps ~= obj.inputDelaySteps
                % Resize history buffer
                if newDelaySteps == 0
                    obj.uHistory = obj.uPrev;
                else
                    obj.uHistory = repmat(obj.uPrev, newDelaySteps + 1, 1);
                end
                obj.inputDelaySteps = newDelaySteps;
            end
        end
        
        function updateTuning(obj, omega_c, omega_o)
            validateattributes(omega_c, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
            validateattributes(omega_o, {'numeric'}, {'vector', 'real', 'finite', 'positive'});
            obj.omega_c = omega_c;
            if isscalar(omega_o)
                obj.omega_o = repmat(omega_o, 1, obj.numObservers);
            elseif numel(omega_o) == obj.numObservers
                obj.omega_o = reshape(omega_o, 1, []);
            else
                error('ADRC:Tuning', ...
                    'omega_o must be scalar or contain one value per observer');
            end
            
            % Recompute gains
            obj.computeESOGains();
            obj.computeControllerGains();
        end
        
        function Xhat = getEstimatedStates(obj)
            Xhat = [obj.getSelectedStates(); obj.getEstimatedDisturbance()];
        end

        function states = getObserverStates(obj, layer)
            if nargin < 2
                states = obj.observerStates;
                return;
            end
            if ~isscalar(layer) || layer < 1 || layer > size(obj.observerStates, 2) || layer ~= floor(layer)
                error('ADRC:ObserverLayer', 'Observer layer must be an integer from 1 to %d', ...
                    size(obj.observerStates, 2));
            end
            states = obj.observerStates(:, layer);
        end
        
        function f = getEstimatedDisturbance(obj)
            f = sum(obj.observerStates(obj.n + 1, :));
        end
    end
    
    methods (Access = private)
        function computeESOGains(obj)
            % Compute discrete-time ESO observer gains based on bandwidth placement
            
            % Continuous-time ESO matrices
            NADRC = obj.controllerOrder;
            A = zeros(NADRC);
            B = zeros(NADRC, 1);
            C = zeros(1, NADRC);
            
            for i = 1:obj.n
                A(i, i+1) = 1;
            end
            B(obj.n) = obj.b0;
            C(1) = 1;
            
            % Discretize using matrix exponential
            Ad_full = expm(A * obj.dT);
            
            % Place each ESO's discrete poles at exp(-omega_o(layer)*dT).
            obj.Ld = zeros(NADRC, obj.numObservers);
            for layer = 1:obj.numObservers
                ZESO = exp(-obj.omega_o(layer) * obj.dT);
                desiredPolynomial = poly(repmat(ZESO, 1, NADRC));
                Adual = Ad_full';
                Bdual = (C * Ad_full)';
                controllability = zeros(NADRC);
                for column = 1:NADRC
                    controllability(:, column) = Adual^(column - 1) * Bdual;
                end
                polynomialMatrix = Adual^NADRC;
                for coefficient = 2:NADRC + 1
                    polynomialMatrix = polynomialMatrix + ...
                        desiredPolynomial(coefficient) * Adual^(NADRC - coefficient + 1);
                end
                selector = zeros(1, NADRC);
                selector(end) = 1;
                obj.Ld(:, layer) = (selector / controllability * polynomialMatrix)';
            end
            
            % Store ESO matrices
            augmentedA = [A, B; zeros(1, NADRC + 1)];
            discreteSystem = expm(augmentedA * obj.dT);
            obj.Bd = discreteSystem(1:NADRC, NADRC + 1);
            obj.Cd = C;
            obj.Beso = zeros(NADRC, obj.numObservers);
            obj.Ad = zeros(NADRC, NADRC, obj.numObservers);
            for layer = 1:obj.numObservers
                obj.Ad(:, :, layer) = Ad_full - obj.Ld(:, layer) * C * Ad_full;
                obj.Beso(:, layer) = obj.Bd - obj.Ld(:, layer) * C * obj.Bd;
            end
        end
        
        function computeControllerGains(obj)
            % Compute state feedback gains for pole placement
            
            obj.K = zeros(obj.n, 1);
            for i = 1:obj.n
                obj.K(obj.n - i + 1) = (factorial(obj.n) / ...
                    (factorial(obj.n - i) * factorial(i))) * obj.omega_c^i;
            end
        end

        function states = getSelectedStates(obj)
            states = zeros(obj.n, 1);
            for state = 1:obj.n
                states(state) = obj.observerStates(state, obj.estimateLayers(state));
            end
        end
        
        function u_sat = saturate(obj, u)
            if ~isfinite(obj.uMin)
                u_sat = min(u, obj.uMax);
            elseif ~isfinite(obj.uMax)
                u_sat = max(u, obj.uMin);
            else
                u_sat = min(max(u, obj.uMin), obj.uMax);
            end
        end
    end
end
