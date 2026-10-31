classdef ADRCMex < handle
    %ADRCMEX MATLAB-compatible wrapper around adrc_codegen_mex.
    %
    % Run build_adrc_mex before constructing this class, or add its output
    % directory to the MATLAB path.

    properties (SetAccess = private)
        controllerOrder
        isInitialized = false
    end

    properties (Access = private)
        n
        instanceId
        config
        estimatedStates = zeros(5, 1)
        observerStates = zeros(5, 4)
        uPrev = 0
    end

    methods
        function obj = ADRCMex(systemOrder)
            if nargin < 1 || ~isscalar(systemOrder) || systemOrder < 1 || ...
                    systemOrder > 4 || systemOrder ~= floor(systemOrder)
                error('ADRC:Constructor', 'System order must be a positive integer from 1 to 4');
            end
            obj.n = systemOrder;
            obj.instanceId = ADRCMex.nextInstanceId();
            obj.controllerOrder = systemOrder + 1;
            obj.config = adrc_codegen_config();
            obj.config.instanceId = obj.instanceId;
            obj.config.n = systemOrder;
        end

        function initialize(obj, varargin)
            p = inputParser;
            addParameter(p, 'omega_c', 4.0);
            addParameter(p, 'omega_o', 40.0);
            addParameter(p, 'Tsettle', []);
            addParameter(p, 'kob', 10);
            addParameter(p, 'b0', 1.0);
            addParameter(p, 'uMin', -inf);
            addParameter(p, 'uMax', inf);
            addParameter(p, 'dT', 0.01);
            addParameter(p, 'XhatInit', zeros(obj.controllerOrder, 1));
            addParameter(p, 'uInit', 0);
            addParameter(p, 'inputDelay', 0);
            addParameter(p, 'TD_method', 'none');
            addParameter(p, 'TD_params', {});
            addParameter(p, 'useCascaded', false); %#ok<INUSD>
            addParameter(p, 'numObservers', 1);
            addParameter(p, 'estimateLayers', []);
            addParameter(p, 'observerGains', []);
            addParameter(p, 'controllerGains', []);
            parse(p, varargin{:});
            r = p.Results;
            if ~isempty(r.Tsettle)
                r.omega_c = 2 * obj.controllerOrder / r.Tsettle;
                r.omega_o = r.omega_c * r.kob;
            elseif ~ismember('kob', p.UsingDefaults) && ismember('omega_o', p.UsingDefaults)
                r.omega_o = r.omega_c * r.kob;
            end
            if r.numObservers < 1 || r.numObservers > 4 || r.numObservers ~= floor(r.numObservers)
                error('ADRC:Initialize', 'numObservers must be an integer from 1 to 4');
            end
            if isempty(r.estimateLayers)
                r.estimateLayers = repmat(r.numObservers, obj.n, 1);
            end
            if numel(r.XhatInit) ~= obj.controllerOrder || numel(r.estimateLayers) ~= obj.n
                error('ADRC:Initialize', 'Initial state and estimate layer sizes do not match the order');
            end
            if isscalar(r.omega_o)
                r.omega_o = repmat(r.omega_o, 1, r.numObservers);
            elseif numel(r.omega_o) ~= r.numObservers
                error('ADRC:Initialize', 'omega_o must be scalar or contain one value per observer');
            end
            obj.config.numObservers = r.numObservers;
            obj.config.dT = r.dT;
            obj.config.omega_c = r.omega_c;
            obj.config.omega_o(:) = 0;
            obj.config.omega_o(1:r.numObservers) = r.omega_o(:);
            obj.config.b0 = r.b0;
            obj.config.uMin = r.uMin;
            obj.config.uMax = r.uMax;
            obj.config.inputDelaySteps = round(r.inputDelay / r.dT);
            obj.config.uInit = r.uInit;
            obj.config.XhatInit(:) = 0;
            obj.config.XhatInit(1:obj.controllerOrder) = r.XhatInit(:);
            obj.config.estimateLayers(:) = 1;
            obj.config.estimateLayers(1:obj.n) = r.estimateLayers(:);
            obj.config.observerGains(:) = 0;
            obj.config.useObserverGains = ~isempty(r.observerGains);
            if obj.config.useObserverGains
                obj.config.observerGains(1:obj.controllerOrder, 1:r.numObservers) = r.observerGains;
            end
            obj.config.controllerGains(:) = 0;
            obj.config.useControllerGains = ~isempty(r.controllerGains);
            if obj.config.useControllerGains
                obj.config.controllerGains(1:obj.n) = r.controllerGains(:);
            end
            obj.config.useTD = ~strcmpi(char(r.TD_method), 'none');
            obj.config.tdMethod = uint8(0);
            if obj.config.useTD
                methods = {'euler', 'tod', 'ld', 'red', 'intd'};
                methodIndex = find(strcmpi(char(r.TD_method), methods), 1);
                if isempty(methodIndex)
                    error('ADRC:Initialize', 'Unknown tracking differentiator method');
                end
                obj.config.tdMethod = uint8(methodIndex);
            end
            obj.config.tdParams(:) = 1;
            if obj.config.useTD && obj.config.tdMethod == 1
                obj.config.tdParams(1) = 0.9;
            end
            if ~isempty(r.TD_params)
                params = cellfun(@double, r.TD_params(:));
                if numel(params) > 4
                    error('ADRC:Initialize', 'TD_params accepts at most four values');
                end
                obj.config.tdParams(1:numel(params)) = params;
            end
            [~, obj.estimatedStates, obj.observerStates, obj.isInitialized] = ...
                adrc_codegen_mex(uint8(0), 0, 0, zeros(4, 1), 0, obj.config);
            obj.uPrev = r.uInit;
        end

        function u = step(obj, reference, output, varargin)
            refDerivs = zeros(4, 1);
            numRefDerivs = 0;
            if ~isempty(varargin)
                if numel(varargin) == 1 && isvector(varargin{1})
                    values = varargin{1}(:);
                else
                    values = cellfun(@double, varargin(:));
                end
                numRefDerivs = min(numel(values), 4);
                refDerivs(1:numRefDerivs) = values(1:numRefDerivs);
            end
            [u, obj.estimatedStates, obj.observerStates, obj.isInitialized] = ...
                adrc_codegen_mex(uint8(1), reference, output, refDerivs, ...
                numRefDerivs, obj.config);
            obj.uPrev = u;
        end

        function reset(obj, XhatInit, uInit)
            if nargin >= 2 && ~isempty(XhatInit)
                obj.config.XhatInit(:) = 0;
                obj.config.XhatInit(1:obj.controllerOrder) = XhatInit(:);
            end
            if nargin >= 3 && ~isempty(uInit)
                obj.config.uInit = uInit;
            end
            [~, obj.estimatedStates, obj.observerStates, obj.isInitialized] = ...
                adrc_codegen_mex(uint8(2), 0, 0, zeros(4, 1), 0, obj.config);
            obj.uPrev = obj.config.uInit;
        end

        function setSaturation(obj, uMin, uMax)
            obj.config.uMin = uMin;
            obj.config.uMax = uMax;
        end

        function setTD(obj, method, varargin)
            obj.config.useTD = ~strcmpi(char(method), 'none');
            obj.config.tdMethod = uint8(0);
            if obj.config.useTD
                methods = {'euler', 'tod', 'ld', 'red', 'intd'};
                methodIndex = find(strcmpi(char(method), methods), 1);
                if isempty(methodIndex)
                    error('ADRC:Initialize', 'Unknown tracking differentiator method');
                end
                obj.config.tdMethod = uint8(methodIndex);
            end
            obj.config.tdParams(:) = 1;
            if obj.config.useTD && obj.config.tdMethod == 1
                obj.config.tdParams(1) = 0.9;
            end
            if ~isempty(varargin)
                params = cellfun(@double, varargin(:));
                if numel(params) > 4
                    error('ADRC:Initialize', 'TD parameters accept at most four values');
                end
                obj.config.tdParams(1:numel(params)) = params;
            end
            [obj.uPrev, obj.estimatedStates, obj.observerStates, obj.isInitialized] = ...
                adrc_codegen_mex(uint8(4), 0, 0, zeros(4, 1), 0, obj.config);
        end

        function setInputDelay(obj, delaySec)
            obj.config.inputDelaySteps = round(delaySec / obj.config.dT);
            [obj.uPrev, obj.estimatedStates, obj.observerStates, obj.isInitialized] = ...
                adrc_codegen_mex(uint8(5), 0, 0, zeros(4, 1), 0, obj.config);
        end

        function updateTuning(obj, omegaC, omegaO)
            obj.config.omega_c = omegaC;
            if isscalar(omegaO)
                obj.config.omega_o(:) = omegaO;
            elseif numel(omegaO) == obj.config.numObservers
                obj.config.omega_o(:) = 0;
                obj.config.omega_o(1:obj.config.numObservers) = omegaO(:);
            else
                error('ADRC:Tuning', 'omega_o must be scalar or contain one value per observer');
            end
            [obj.uPrev, obj.estimatedStates, obj.observerStates, obj.isInitialized] = ...
                adrc_codegen_mex(uint8(6), 0, 0, zeros(4, 1), 0, obj.config);
        end

        function Xhat = getEstimatedStates(obj)
            obj.refresh();
            Xhat = obj.estimatedStates(1:obj.controllerOrder);
        end

        function f = getEstimatedDisturbance(obj)
            obj.refresh();
            f = obj.estimatedStates(obj.controllerOrder);
        end

        function states = getObserverStates(obj, layer)
            obj.refresh();
            if nargin < 2
                states = obj.observerStates(1:obj.controllerOrder, 1:obj.config.numObservers);
            else
                states = obj.observerStates(1:obj.controllerOrder, layer);
            end
        end
    end

    methods (Access = private)
        function refresh(obj)
            [obj.uPrev, obj.estimatedStates, obj.observerStates, obj.isInitialized] = ...
                adrc_codegen_mex(uint8(3), 0, 0, zeros(4, 1), 0, obj.config);
        end
    end

    methods (Static, Access = private)
        function id = nextInstanceId()
            persistent nextId
            if isempty(nextId)
                nextId = 0;
            end
            nextId = nextId + 1;
            if nextId > 16
                error('ADRC:CodegenConfig', 'At most 16 ADRCMex instances are supported');
            end
            id = nextId;
        end
    end
end
