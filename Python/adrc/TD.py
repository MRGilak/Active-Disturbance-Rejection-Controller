import numpy as np

class TD:
    def __init__(self, method, dt, *params):
        if dt <= 0:
            raise ValueError("Sample time dt must be positive")
        
        self.method = method.lower()
        self.dt = dt
        self.params = list(params)
        self.state = {}
        
        # Output properties
        self.y = 0.0      # Filtered output
        self.yd = 0.0     # First derivative
        self.ydd = 0.0    # Second derivative
        
        # Initialize method-specific state
        self._initialize_state(0.0)
    
    def _initialize_state(self, initial_value):
        if self.method == 'euler':
            self.state['prev_y'] = initial_value
            self.state['prev_yd'] = 0.0
            self.y = initial_value
            self.yd = 0.0
            self.ydd = 0.0
            
        elif self.method == 'tod':
            self.state['v1'] = initial_value
            self.state['v2'] = 0.0
            self.y = initial_value
            self.yd = 0.0
            self.ydd = 0.0
            
        elif self.method == 'ld':
            self.state['v1'] = initial_value
            self.state['v2'] = 0.0
            self.y = initial_value
            self.yd = 0.0
            self.ydd = 0.0
            
        elif self.method == 'red':
            self.state['v1'] = initial_value
            self.state['v2'] = 0.0
            self.y = initial_value
            self.yd = 0.0
            self.ydd = 0.0
            
        elif self.method == 'intd':
            self.state['v1'] = initial_value
            self.state['v2'] = 0.0
            self.y = initial_value
            self.yd = 0.0
            self.ydd = 0.0
            
        else:
            raise ValueError(f"Unknown TD method: {self.method}")
    
    def update(self, ref):
        if self.method == 'euler':
            self._update_euler(ref)
        elif self.method == 'tod':
            self._update_tod(ref)
        elif self.method == 'ld':
            self._update_ld(ref)
        elif self.method == 'red':
            self._update_red(ref)
        elif self.method == 'intd':
            self._update_intd(ref)
    
    def reset(self, initial_value=0.0):
        self._initialize_state(initial_value)
    
    def set_parameters(self, *params):
        self.params = list(params)
    
    def set_sample_time(self, dt):
        if dt <= 0:
            raise ValueError("Sample time dt must be positive")
        self.dt = dt
    
    def _update_euler(self, ref):
        """
        Euler-based low-pass filter with derivative estimation.
        Parameter: a (filter coefficient, default 0.9)
        """
        a = self.params[0] if self.params else 0.9
        
        # Low pass filter
        self.y = a * self.state['prev_y'] + (1 - a) * ref
        
        # Euler approximation for first derivative
        self.yd = (self.y - self.state['prev_y']) / self.dt
        
        # Euler approximation for second derivative
        self.ydd = (self.yd - self.state['prev_yd']) / self.dt
        
        # Update state
        self.state['prev_y'] = self.y
        self.state['prev_yd'] = self.yd
    
    def _update_tod(self, ref):
        """
        Time Optimal Differentiator.
        Parameter: r (convergence rate, default 1)
        """
        r = self.params[0] if self.params else 1.0
        
        # Compute derivatives
        dot_v1 = self.state['v2']
        dot_v2 = -r * np.sign(self.state['v1'] - ref + 
                               (self.state['v2'] * abs(self.state['v2'])) / (2 * r))
        
        # Forward Euler integration
        self.state['v1'] = self.state['v1'] + dot_v1 * self.dt
        self.state['v2'] = self.state['v2'] + dot_v2 * self.dt
        
        # Outputs
        self.y = self.state['v1']
        self.yd = self.state['v2']
        self.ydd = 0.0  # Second derivative not estimated
    
    def _update_ld(self, ref):
        """
        Linear Differentiator.
        Parameter: lambda (bandwidth parameter, default 1)
        """
        lambda_param = self.params[0] if self.params else 1.0
        
        # Compute derivatives
        dot_v1 = self.state['v2'] - (self.state['v1'] - ref) / lambda_param
        dot_v2 = -2 * (self.state['v1'] - ref) / lambda_param
        
        # Forward Euler integration
        self.state['v1'] = self.state['v1'] + dot_v1 * self.dt
        self.state['v2'] = self.state['v2'] + dot_v2 * self.dt
        
        # Outputs
        self.y = self.state['v1']
        self.yd = self.state['v2']
        self.ydd = 0.0  # Second derivative not estimated
    
    def _update_red(self, ref):
        """
        Robust Exact Differentiator.
        Parameters: lambda1, lambda2 (both default to 1)
        """
        if len(self.params) < 2:
            lambda1 = 1.0
            lambda2 = 1.0
        else:
            lambda1 = self.params[0]
            lambda2 = self.params[1]
        
        # Compute derivatives
        err = self.state['v1'] - ref
        dot_v1 = self.state['v2'] - lambda1 * (abs(err)**0.5) * np.sign(err)
        dot_v2 = -lambda2 * np.sign(err)
        
        # Forward Euler integration
        self.state['v1'] = self.state['v1'] + dot_v1 * self.dt
        self.state['v2'] = self.state['v2'] + dot_v2 * self.dt
        
        # Outputs
        self.y = self.state['v1']
        self.yd = self.state['v2']
        self.ydd = 0.0  # Second derivative not estimated
    
    def _update_intd(self, ref):
        """
        Improved Nonlinear Tracking Differentiator.
        Parameters: alpha, beta, gamma, r (defaults: 1, 1, 1, 1)
        """
        if len(self.params) < 4:
            alpha = 1.0
            beta = 1.0
            gamma = 1.0
            r = 1.0
        else:
            alpha = self.params[0]
            beta = self.params[1]
            gamma = self.params[2]
            r = self.params[3]
        
        # Parameter validation
        if alpha < 0 or alpha > 1:
            import warnings
            warnings.warn('alpha should be between 0 and 1 for INTD method, clamping')
            alpha = max(0.0, min(1.0, alpha))
        if beta <= 0 or gamma <= 0 or r <= 0:
            raise ValueError('beta, gamma, and r must be positive for INTD method')
        
        # Compute derivatives
        dot_v1 = self.state['v2']
        dot_v2 = -r**2 * np.tanh((beta * self.state['v1'] - (1 - alpha) * ref) / gamma) - r * self.state['v2']
        
        # Forward Euler integration
        self.state['v1'] = self.state['v1'] + dot_v1 * self.dt
        self.state['v2'] = self.state['v2'] + dot_v2 * self.dt
        
        # Outputs
        self.y = self.state['v1']
        self.yd = self.state['v2']
        self.ydd = 0.0  # Second derivative not estimated
