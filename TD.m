classdef TD < handle
    properties (Access = private)
        method      char
        dT          double
        params      cell
        state       struct
    end
    
    properties (SetAccess = private)
        y           double = 0    % Filtered output
        yd          double = 0    % First derivative
        ydd         double = 0    % Second derivative
    end
    
    methods
        function obj = TD(method, dT, varargin)
            if nargin < 2
                error('TD requires at least method and dT arguments');
            end
            
            obj.method = lower(method);
            obj.dT = dT;
            obj.params = varargin;
            obj.state = struct();
            
            % Initialize method-specific state
            obj.initializeState(0);
        end
        
        function initializeState(obj, initialValue)
            switch obj.method
                case 'euler'
                    obj.state.prev_y = initialValue;
                    obj.state.prev_yd = 0;
                    obj.y = initialValue;
                    obj.yd = 0;
                    obj.ydd = 0;
                    
                case 'tod'
                    obj.state.v1 = initialValue;
                    obj.state.v2 = 0;
                    obj.y = initialValue;
                    obj.yd = 0;
                    obj.ydd = 0;
                    
                case 'ld'
                    obj.state.v1 = initialValue;
                    obj.state.v2 = 0;
                    obj.y = initialValue;
                    obj.yd = 0;
                    obj.ydd = 0;
                    
                case 'red'
                    obj.state.v1 = initialValue;
                    obj.state.v2 = 0;
                    obj.y = initialValue;
                    obj.yd = 0;
                    obj.ydd = 0;
                    
                case 'intd'
                    obj.state.v1 = initialValue;
                    obj.state.v2 = 0;
                    obj.y = initialValue;
                    obj.yd = 0;
                    obj.ydd = 0;
                    
                otherwise
                    error('Unknown TD method: %s', obj.method);
            end
        end
        
        function update(obj, ref)
            switch obj.method
                case 'euler'
                    obj.updateEuler(ref);
                case 'tod'
                    obj.updateTOD(ref);
                case 'ld'
                    obj.updateLD(ref);
                case 'red'
                    obj.updateRED(ref);
                case 'intd'
                    obj.updateINTD(ref);
            end
        end
        
        function reset(obj, initialValue)
            % Reset TD to initial state
            if nargin < 2
                initialValue = 0;
            end
            obj.initializeState(initialValue);
        end
        
        function setParameters(obj, varargin)
            % Update tuning parameters
            obj.params = varargin;
        end
        
        function setSampleTime(obj, dT)
            % Update sample time
            obj.dT = dT;
        end
    end
    
    methods (Access = private)
        function updateEuler(obj, ref)
            % Euler-based low-pass filter with derivative estimation
            % Parameter: a (filter coefficient, default 0.9)
            
            if isempty(obj.params)
                a = 0.9;
            else
                a = obj.params{1};
            end
            
            % Low pass filter
            obj.y = a * obj.state.prev_y + (1 - a) * ref;
            
            % Euler approximation for first derivative
            obj.yd = (obj.y - obj.state.prev_y) / obj.dT;
            
            % Euler approximation for second derivative
            obj.ydd = (obj.yd - obj.state.prev_yd) / obj.dT;
            
            % Update state
            obj.state.prev_y = obj.y;
            obj.state.prev_yd = obj.yd;
        end
        
        function updateTOD(obj, ref)
            % Time Optimal Differentiator
            % Parameter: r (convergence rate, default 1)
            
            if isempty(obj.params)
                r = 1;
            else
                r = obj.params{1};
            end
            
            % Compute derivatives
            dot_v1 = obj.state.v2;
            dot_v2 = -r * sign(obj.state.v1 - ref + (obj.state.v2 * abs(obj.state.v2)) / (2 * r));
            
            % Forward Euler integration
            obj.state.v1 = obj.state.v1 + dot_v1 * obj.dT;
            obj.state.v2 = obj.state.v2 + dot_v2 * obj.dT;
            
            % Outputs
            obj.y = obj.state.v1;
            obj.yd = obj.state.v2;
            obj.ydd = 0;  % Second derivative not estimated
        end
        
        function updateLD(obj, ref)
            % Linear Differentiator
            % Parameter: lambda (bandwidth parameter, default 1)
            
            if isempty(obj.params)
                lambda = 1;
            else
                lambda = obj.params{1};
            end
            
            % Compute derivatives
            dot_v1 = obj.state.v2 - (obj.state.v1 - ref) / lambda;
            dot_v2 = -2 * (obj.state.v1 - ref) / lambda;
            
            % Forward Euler integration
            obj.state.v1 = obj.state.v1 + dot_v1 * obj.dT;
            obj.state.v2 = obj.state.v2 + dot_v2 * obj.dT;
            
            % Outputs
            obj.y = obj.state.v1;
            obj.yd = obj.state.v2;
            obj.ydd = 0;  % Second derivative not estimated
        end
        
        function updateRED(obj, ref)
            % Robust Exact Differentiator
            % Parameters: lambda1, lambda2 (both default to 1)
            
            if length(obj.params) < 2
                lambda1 = 1;
                lambda2 = 1;
            else
                lambda1 = obj.params{1};
                lambda2 = obj.params{2};
            end
            
            % Compute derivatives
            err = obj.state.v1 - ref;
            dot_v1 = obj.state.v2 - lambda1 * (abs(err)^0.5) * sign(err);
            dot_v2 = -lambda2 * sign(err);
            
            % Forward Euler integration
            obj.state.v1 = obj.state.v1 + dot_v1 * obj.dT;
            obj.state.v2 = obj.state.v2 + dot_v2 * obj.dT;
            
            % Outputs
            obj.y = obj.state.v1;
            obj.yd = obj.state.v2;
            obj.ydd = 0;  % Second derivative not estimated
        end
        
        function updateINTD(obj, ref)
            % Improved Nonlinear Tracking Differentiator
            % Parameters: alpha, beta, gamma, r (defaults: 1, 1, 1, 1)
            
            if length(obj.params) < 4
                alpha = 1;
                beta = 1;
                gamma = 1;
                r = 1;
            else
                alpha = obj.params{1};
                beta = obj.params{2};
                gamma = obj.params{3};
                r = obj.params{4};
            end
            
            % Parameter validation
            if alpha < 0 || alpha > 1
                warning('alpha should be between 0 and 1 for INTD method, clamping');
                alpha = max(0, min(1, alpha));
            end
            if beta <= 0 || gamma <= 0 || r <= 0
                error('beta, gamma, and r must be positive for INTD method');
            end
            
            % Compute derivatives
            dot_v1 = obj.state.v2;
            dot_v2 = -r^2 * tanh((beta * obj.state.v1 - (1 - alpha) * ref) / gamma) - r * obj.state.v2;
            
            % Forward Euler integration
            obj.state.v1 = obj.state.v1 + dot_v1 * obj.dT;
            obj.state.v2 = obj.state.v2 + dot_v2 * obj.dT;
            
            % Outputs
            obj.y = obj.state.v1;
            obj.yd = obj.state.v2;
            obj.ydd = 0;  % Second derivative not estimated
        end
    end
end
