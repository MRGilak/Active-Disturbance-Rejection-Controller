import numpy as np
import math
from scipy.linalg import expm
from TD import TD

class ADRC:
    def __init__(self, system_order):
        if system_order < 1 or system_order > 4:
            raise ValueError('System order must be between 1 and 4')
        
        self.n = int(system_order)
        self.controller_order = self.n + 1
        
        # State variables
        self.Xhat = np.zeros(self.controller_order)
        self.u_prev = 0.0
        self.Ke = 1.0
        
        # Gains and matrices (initialized in initialize())
        self.Ld = None
        self.K = None
        self.Ad = None
        self.Bd = None
        self.Cd = None
        
        # Configuration parameters (set in initialize())
        self.b0 = None
        self.Tsettle = None
        self.kob = None
        self.dt = None
        
        # Saturation limits
        self.u_min = -np.inf
        self.u_max = np.inf
        
        # Input delay handling
        self.input_delay_steps = 0
        self.u_history = np.array([0.0])
        
        # Tracking differentiator (optional)
        self.use_td = False
        self.TD_obj = TD('euler', 0.01, 0.9)  # Dummy initialization
        
        # Initialization flag
        self.is_initialized = False
    
    def initialize(self, Tsettle=1.0, kob=10, b0=1.0, u_min=-np.inf, u_max=np.inf,
                   dt=0.01, Xhat_init=None, u_init=0.0, Ke=1.0, input_delay=0.0,
                   TD_method='none', TD_params=None):
        if Tsettle <= 0:
            raise ValueError("Tsettle must be positive")
        if kob <= 0:
            raise ValueError("kob must be positive")
        if dt <= 0:
            raise ValueError("dt must be positive")
        if Ke <= 0:
            raise ValueError("Ke must be positive")
        if input_delay < 0:
            raise ValueError("input_delay must be non-negative")
        
        # Store parameters
        self.Tsettle = Tsettle
        self.kob = kob
        self.b0 = b0 / Ke
        self.Ke = Ke
        self.dt = dt
        self.u_min = u_min
        self.u_max = u_max
        
        # Initialize state
        if Xhat_init is None:
            Xhat_init = np.zeros(self.controller_order)
        else:
            Xhat_init = np.asarray(Xhat_init).flatten()
            if len(Xhat_init) != self.controller_order:
                raise ValueError(f'Xhat_init must have length {self.controller_order}')
        
        self.Xhat = Xhat_init.copy()
        self.u_prev = u_init
        
        # Configure input delay
        self.input_delay_steps = max(0, int(round(input_delay / dt)))
        self.u_history = np.full(self.input_delay_steps + 1, u_init)
        
        # Compute ESO gains and matrices
        self._compute_eso_gains()
        self._compute_controller_gains()
        
        # Setup TD if requested
        TD_method = TD_method.lower()
        if TD_method != 'none':
            self.use_td = True
            if TD_params is None:
                TD_params = []
            self.TD_obj = TD(TD_method, dt, *TD_params)
            self.TD_obj.reset(0.0)
        else:
            self.use_td = False
            self.TD_obj = TD('euler', dt, 0.9)
        
        self.is_initialized = True
    
    def step(self, reference, output, *ref_derivatives):
        if not self.is_initialized:
            raise RuntimeError('Controller must be initialized before use')
        
        # Update TD if enabled
        if self.use_td:
            self.TD_obj.update(reference)
            ref_filtered = self.TD_obj.y
            ref_derivs = [self.TD_obj.yd]
            if self.n >= 2:
                ref_derivs.append(self.TD_obj.ydd)
        else:
            ref_filtered = reference
            ref_derivs = []
        
        # Parse optional reference derivatives
        if ref_derivatives:
            if len(ref_derivatives) == 1 and hasattr(ref_derivatives[0], '__iter__'):
                # Single array-like argument
                ref_derivs = list(ref_derivatives[0])
            else:
                # Multiple scalar arguments
                ref_derivs = list(ref_derivatives)
        
        # Update ESO (using delayed control input)
        u_for_eso = self.u_history[0]
        self.Xhat = self.Ad @ self.Xhat + self.Bd * u_for_eso + self.Ld * output
        
        # Build reference state vector
        r_states = np.zeros(self.n)
        r_states[0] = ref_filtered
        
        if ref_derivs:
            m = min(self.n - 1, len(ref_derivs))
            if m > 0:
                r_states[1:1+m] = ref_derivs[:m]
        
        # nth derivative for feedforward
        ref_nth = 0.0
        if len(ref_derivs) >= self.n:
            ref_nth = ref_derivs[self.n - 1]
        
        # Extended reference vector [r, r', ..., r^(n)]
        RefVec = np.concatenate([r_states, [ref_nth]])
        
        # Control law with disturbance rejection
        K_extended = np.concatenate([self.K, [1.0]])
        u = np.dot(K_extended, RefVec - self.Xhat) / self.b0
        
        # Apply saturation
        u = self._saturate(u)
        
        # Update control history for delay emulation
        if len(self.u_history) > 1:
            self.u_history[:-1] = self.u_history[1:]
            self.u_history[-1] = u
        else:
            self.u_history[0] = u
        
        self.u_prev = u
        return u
    
    def reset(self, Xhat_init=None, u_init=None):
        if Xhat_init is None:
            Xhat_init = np.zeros(self.controller_order)
        else:
            Xhat_init = np.asarray(Xhat_init).flatten()
        
        if u_init is None:
            u_init = 0.0
        
        self.Xhat = Xhat_init.copy()
        self.u_prev = u_init
        self.u_history = np.full(self.input_delay_steps + 1, u_init)
        
        if self.use_td:
            self.TD_obj.reset(0.0)
    
    def set_td(self, method, *params):
        method = method.lower()
        if method == 'none':
            self.use_td = False
        else:
            self.use_td = True
            self.TD_obj = TD(method, self.dt, *params)
            self.TD_obj.reset(0.0)
    
    def set_saturation(self, u_min, u_max):
        self.u_min = u_min
        self.u_max = u_max
    
    def set_input_delay(self, delay_sec):
        new_delay_steps = max(0, int(round(delay_sec / self.dt)))
        
        if new_delay_steps != self.input_delay_steps:
            # Resize history buffer
            if new_delay_steps == 0:
                self.u_history = np.array([self.u_prev])
            else:
                self.u_history = np.full(new_delay_steps + 1, self.u_prev)
            self.input_delay_steps = new_delay_steps
    
    def update_tuning(self, Tsettle, kob):
        self.Tsettle = Tsettle
        self.kob = kob
        
        # Recompute gains
        self._compute_eso_gains()
        self._compute_controller_gains()
    
    def get_estimated_states(self):
        return self.Xhat.copy()
    
    def get_estimated_disturbance(self):
        return self.Xhat[-1]
    
    def _compute_eso_gains(self):
        # Continuous-time ESO matrices
        NADRC = self.controller_order
        A = np.zeros((NADRC, NADRC))
        B = np.zeros(NADRC)
        C = np.zeros(NADRC)
        
        for i in range(self.n):
            A[i, i+1] = 1.0
        B[self.n - 1] = self.b0
        C[0] = 1.0
        
        # Discretize using matrix exponential
        Ad_full = expm(A * self.dt)
        I = np.eye(NADRC)
        
        # Compute Bd using integration
        sum_matrix = np.zeros((NADRC, NADRC))
        for i in range(1, self.n + 1):
            sum_matrix = sum_matrix + np.linalg.matrix_power(A, i-1) * (self.dt**i) / math.factorial(i)
        Bd_full = sum_matrix @ B
        
        # Observer pole placement
        if self.n == 1:
            Scl = -4.0 / self.Tsettle
            SESO = self.kob * Scl
            ZESO = np.exp(SESO * self.dt)
            Ld = np.zeros(2)
            Ld[0] = 1 - ZESO**2
            Ld[1] = (1 - ZESO)**2 / self.dt
            
        elif self.n == 2:
            Scl = -6.0 / self.Tsettle
            SESO = self.kob * Scl
            ZESO = np.exp(SESO * self.dt)
            Ld = np.zeros(3)
            Ld[0] = 1 - ZESO**3
            Ld[1] = (1 + ZESO) * ((1 - ZESO)**2) * 3 / (2 * self.dt)
            Ld[2] = ((1 - ZESO)**3) / (self.dt**2)
            
        elif self.n == 3:
            Scl = -8.0 / self.Tsettle
            SESO = self.kob * Scl
            ZESO = np.exp(SESO * self.dt)
            Ld = np.zeros(4)
            Ld[0] = 1 - ZESO**4
            Ld[1] = ((1 - ZESO)**2) * (11 + ZESO * (14 + 11 * ZESO)) / (6 * self.dt)
            Ld[2] = ((1 - ZESO)**3) * (1 + ZESO) * 2 / (self.dt**2)
            Ld[3] = ((1 - ZESO)**4) / (self.dt**3)
            
        elif self.n == 4:
            Scl = -10.0 / self.Tsettle
            SESO = self.kob * Scl
            ZESO = np.exp(SESO * self.dt)
            Ld = np.zeros(5)
            Ld[0] = 1 - ZESO**5
            Ld[1] = ((1 - ZESO)**2) * (1 + ZESO) * (5 + ZESO * (2 + 5 * ZESO)) * 5 / (12 * self.dt)
            Ld[2] = ((1 - ZESO)**3) * (7 + ZESO * (10 + 7 * ZESO)) * 5 / (12 * self.dt**2)
            Ld[3] = ((1 - ZESO)**4) * (1 + ZESO) * 5 / (2 * self.dt**3)
            Ld[4] = ((1 - ZESO)**5) / (self.dt**4)
        
        # Store ESO matrices
        self.Ld = Ld
        self.Ad = Ad_full - np.outer(Ld, C @ Ad_full)
        self.Bd = Bd_full - Ld * (C @ Bd_full)
        self.Cd = C
    
    def _compute_controller_gains(self):
        Scl = -4.0 / self.Tsettle  # Base scaling for controller
        
        self.K = np.zeros(self.n)
        for i in range(1, self.n + 1):
            self.K[self.n - i] = (math.factorial(self.n) / 
                                   (math.factorial(self.n - i) * math.factorial(i))) * (-Scl)**i
    
    def _saturate(self, u):
        if not np.isfinite(self.u_min):
            return min(u, self.u_max)
        elif not np.isfinite(self.u_max):
            return max(u, self.u_min)
        else:
            return np.clip(u, self.u_min, self.u_max)
