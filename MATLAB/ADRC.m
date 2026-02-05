classdef ADRC < handle
    properties (Access = private)
        % System configuration
        n               double {mustBeInteger, mustBePositive}  % System order
        dT              double {mustBePositive}                 % Sample time
        
        % Gains and matrices
        Ld              double                                  % Observer gain vector
        K               double                                  % Controller gain vector
        Ad              double                                  % Discrete ESO A matrix
        Bd              double                                  % Discrete ESO B matrix
        Cd              double                                  % Discrete ESO C matrix
        
        % ESO configuration
        b0              double                                  % Control gain estimate
        Tsettle         double                                  % Desired settling time
        kob             double                                  % Observer bandwidth multiplier
        Ke              double                                  % Error gain
        
        % State variables
        Xhat            double                                  % Extended state estimates [x1...xn, f]
        uPrev           double                                  % Previous control input
        
        % Saturation
        uMin            double = -inf                           % Lower saturation limit
        uMax            double = inf                            % Upper saturation limit
        
        % Input delay handling
        inputDelaySteps double {mustBeInteger} = 0              % Delay steps for ESO
        uHistory        double                                  % Control history buffer
        
        % Tracking differentiator (optional)
        useTD           logical = false                         % Enable TD
        TD_obj          TD                                      % TD object instance
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
            
            if systemOrder < 1 || systemOrder > 4
                error('ADRC:Constructor', 'System order must be between 1 and 4');
            end
            
            obj.n = systemOrder;
            obj.controllerOrder = systemOrder + 1;
            obj.Xhat = zeros(obj.controllerOrder, 1);
            obj.uPrev = 0;
            obj.Ke = 1;
            
            % Initialize TD_obj with a dummy object 
            obj.TD_obj = TD('euler', 0.01, 0.9);
        end
        
        function initialize(obj, varargin)
            % Parse inputs
            p = inputParser;
            addParameter(p, 'Tsettle', 1.0, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'kob', 10, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'b0', 1.0, @(x) isnumeric(x) && isscalar(x));
            addParameter(p, 'uMin', -inf, @(x) isnumeric(x) && isscalar(x));
            addParameter(p, 'uMax', inf, @(x) isnumeric(x) && isscalar(x));
            addParameter(p, 'dT', 0.01, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'XhatInit', zeros(obj.controllerOrder, 1), @(x) isnumeric(x) && isvector(x));
            addParameter(p, 'uInit', 0, @(x) isnumeric(x) && isscalar(x));
            addParameter(p, 'Ke', 1, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(p, 'inputDelay', 0, @(x) isnumeric(x) && isscalar(x) && x >= 0);
            addParameter(p, 'TD_method', 'none', @(x) ischar(x) || isstring(x));
            addParameter(p, 'TD_params', {}, @iscell);
            
            parse(p, varargin{:});
            
            % Store parameters
            obj.Tsettle = p.Results.Tsettle;
            obj.kob = p.Results.kob;
            obj.b0 = p.Results.b0 / p.Results.Ke;
            obj.Ke = p.Results.Ke;
            obj.dT = p.Results.dT;
            obj.uMin = p.Results.uMin;
            obj.uMax = p.Results.uMax;
            
            % Initialize state
            XhatInit = p.Results.XhatInit(:);
            if length(XhatInit) ~= obj.controllerOrder
                error('ADRC:Initialize', 'XhatInit must have length %d', obj.controllerOrder);
            end
            obj.Xhat = XhatInit;
            obj.uPrev = p.Results.uInit;
            
            % Configure input delay
            obj.inputDelaySteps = max(0, round(p.Results.inputDelay / obj.dT));
            obj.uHistory = repmat(p.Results.uInit, obj.inputDelaySteps + 1, 1);
            
            % Compute ESO gains and matrices
            obj.computeESOGains();
            obj.computeControllerGains();
            
            % Setup TD if requested
            TD_method = lower(p.Results.TD_method);
            if ~strcmp(TD_method, 'none')
                obj.useTD = true;
                obj.TD_obj = TD(TD_method, obj.dT, p.Results.TD_params{:});
                obj.TD_obj.reset(0);
            else
                obj.useTD = false;
                % Create a default TD object
                obj.TD_obj = TD('euler', obj.dT, 0.9);
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
                if numel(varargin) == 1 && isvector(varargin{1})
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
            obj.Xhat = obj.Ad * obj.Xhat + obj.Bd * u_for_eso + obj.Ld * output;
            
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
            
            % Extended reference vector [r, r', ..., r^(n)]
            RefVec = [r_states; ref_nth];
            
            % Control law with disturbance rejection
            u = dot([obj.K; 1], RefVec - obj.Xhat) / obj.b0;
            
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
            
            obj.Xhat = XhatInit(:);
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
                    obj.TD_obj = TD(method, obj.dT, varargin{:});
                else
                    % Update existing TD - need to recreate with new method
                    obj.TD_obj = TD(method, obj.dT, varargin{:});
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
        
        function updateTuning(obj, Tsettle, kob)
            obj.Tsettle = Tsettle;
            obj.kob = kob;
            
            % Recompute gains
            obj.computeESOGains();
            obj.computeControllerGains();
        end
        
        function Xhat = getEstimatedStates(obj)
            Xhat = obj.Xhat;
        end
        
        function f = getEstimatedDisturbance(obj)
            f = obj.Xhat(end);
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
            I = eye(NADRC);
            
            % Compute Bd using integration
            sum_matrix = zeros(NADRC);
            for i = 1:obj.n
                sum_matrix = sum_matrix + (A^(i-1)) * (obj.dT^i) / factorial(i);
            end
            Bd_full = sum_matrix * B;
            
            % Observer pole placement
            switch obj.n
                case 1
                    Scl = -4 / obj.Tsettle;
                    SESO = obj.kob * Scl;
                    ZESO = exp(SESO * obj.dT);
                    Ld = zeros(2, 1);
                    Ld(1) = 1 - (ZESO)^2;
                    Ld(2) = (1 - ZESO)^2 / obj.dT;
                    
                case 2
                    Scl = -6 / obj.Tsettle;
                    SESO = obj.kob * Scl;
                    ZESO = exp(SESO * obj.dT);
                    Ld = zeros(3, 1);
                    Ld(1) = 1 - (ZESO)^3;
                    Ld(2) = (1 + ZESO) * ((1 - ZESO)^2) * 3 / (2 * obj.dT);
                    Ld(3) = ((1 - ZESO)^3) / (obj.dT^2);
                    
                case 3
                    Scl = -8 / obj.Tsettle;
                    SESO = obj.kob * Scl;
                    ZESO = exp(SESO * obj.dT);
                    Ld = zeros(4, 1);
                    Ld(1) = 1 - (ZESO)^4;
                    Ld(2) = ((1 - ZESO)^2) * (11 + ZESO * (14 + 11 * ZESO)) / (6 * obj.dT);
                    Ld(3) = ((1 - ZESO)^3) * (1 + ZESO) * 2 / (obj.dT^2);
                    Ld(4) = ((1 - ZESO)^4) / (obj.dT^3);
                    
                case 4
                    Scl = -10 / obj.Tsettle;
                    SESO = obj.kob * Scl;
                    ZESO = exp(SESO * obj.dT);
                    Ld = zeros(5, 1);
                    Ld(1) = 1 - (ZESO)^5;
                    Ld(2) = ((1 - ZESO)^2) * (1 + ZESO) * (5 + ZESO * (2 + 5 * ZESO)) * 5 / (12 * obj.dT);
                    Ld(3) = ((1 - ZESO)^3) * (7 + ZESO * (10 + 7 * ZESO)) * 5 / (12 * obj.dT^2);
                    Ld(4) = ((1 - ZESO)^4) * (1 + ZESO) * 5 / (2 * obj.dT^3);
                    Ld(5) = ((1 - ZESO)^5) / (obj.dT^4);
            end
            
            % Store ESO matrices
            obj.Ld = Ld;
            obj.Ad = Ad_full - Ld * C * Ad_full;
            obj.Bd = Bd_full - Ld * C * Bd_full;
            obj.Cd = C;
        end
        
        function computeControllerGains(obj)
            % Compute state feedback gains for pole placement
            
            % Closed-loop poles
            Scl = -(2 * (obj.n + 1)) / obj.Tsettle;
            
            obj.K = zeros(obj.n, 1);
            for i = 1:obj.n
                obj.K(obj.n - i + 1) = (factorial(obj.n) / (factorial(obj.n - i) * factorial(i))) * (-Scl)^i;
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
