import math

import numpy as np
from scipy.linalg import expm

from .TD import TD

class ADRC:
    def __init__(self, system_order):
        if system_order < 1:
            raise ValueError("System order must be a positive integer")
        self.n = int(system_order)
        self.controller_order = self.n + 1
        self.observerStates = np.zeros((self.controller_order, 1))
        self.uPrev = 0.0
        self.Ld = self.K = self.Ad = self.Bd = self.Cd = None
        self.Beso = None
        self.b0 = self.omega_c = self.omega_o = self.dT = None
        self.numObservers = 1
        self.estimateLayers = np.ones(self.n, dtype=int)
        self.uMin, self.uMax = -np.inf, np.inf
        self.inputDelaySteps = 0
        self.uHistory = np.array([0.0])
        self.useTD = False
        self.TD_obj = TD("euler", 0.01, 0.9)
        self.useCascaded = False
        self.isInitialized = False

    def initialize(self, omega_c=4.0, omega_o=40.0, Tsettle=None, kob=10,
                   b0=1.0, uMin=-np.inf, uMax=np.inf, dT=0.01,
                   XhatInit=None, uInit=0.0, inputDelay=0.0,
                   TD_method="none", TD_params=None, useCascaded=False,
                   numObservers=1, estimateLayers=None, observerGains=None,
                   controllerGains=None, **legacy):
        # Keep the pre-generalization spelling usable by existing callers.
        if "dt" in legacy:
            dT = legacy.pop("dt")
        if "Xhat_init" in legacy:
            XhatInit = legacy.pop("Xhat_init")
        uMin = legacy.pop("u_min", uMin)
        uMax = legacy.pop("u_max", uMax)
        uInit = legacy.pop("u_init", uInit)
        inputDelay = legacy.pop("input_delay", inputDelay)
        useCascaded = legacy.pop("use_cascaded", useCascaded)
        Ke = legacy.pop("Ke", 1.0)
        if legacy:
            raise TypeError("Unexpected parameter(s): " + ", ".join(legacy))
        if Tsettle is not None:
            if Tsettle <= 0:
                raise ValueError("Tsettle must be positive")
            omega_c = 2.0 * (self.n + 1) / Tsettle
            if "omega_o" not in legacy:
                omega_o = omega_c * kob
        if dT <= 0 or numObservers < 1 or int(numObservers) != numObservers:
            raise ValueError("dT must be positive and numObservers must be a positive integer")
        if omega_c <= 0 or np.any(np.asarray(omega_o) <= 0) or kob <= 0:
            raise ValueError("Bandwidths and kob must be positive")
        if inputDelay < 0:
            raise ValueError("inputDelay must be non-negative")

        if Ke <= 0:
            raise ValueError("Ke must be positive")
        self.omega_c, self.dT, self.b0 = float(omega_c), float(dT), float(b0) / Ke
        self.numObservers = int(numObservers)
        self.omega_o = np.full(self.numObservers, omega_o, dtype=float) if np.isscalar(omega_o) else np.asarray(omega_o, dtype=float).reshape(-1)
        if len(self.omega_o) != self.numObservers:
            raise ValueError("omega_o must be scalar or contain one value per observer")
        self.uMin, self.uMax = uMin, uMax
        self.useCascaded = bool(useCascaded or self.numObservers > 1)
        self.estimateLayers = np.full(self.n, self.numObservers, dtype=int) if estimateLayers is None else np.asarray(estimateLayers, dtype=int).reshape(-1)
        if len(self.estimateLayers) != self.n or np.any(self.estimateLayers < 1) or np.any(self.estimateLayers > self.numObservers):
            raise ValueError(f"estimateLayers must contain {self.n} values between 1 and numObservers")
        if XhatInit is None:
            XhatInit = np.zeros(self.controller_order)
        XhatInit = np.asarray(XhatInit, dtype=float).reshape(-1)
        if len(XhatInit) != self.controller_order:
            raise ValueError(f"XhatInit must have length {self.controller_order}")
        self.observerStates = np.tile(XhatInit[:, None], (1, self.numObservers))
        self.uPrev = float(uInit)
        self.inputDelaySteps = max(0, int(round(inputDelay / self.dT)))
        self.uHistory = np.full(self.inputDelaySteps + 1, uInit, dtype=float)
        self._compute_eso_gains(observerGains)
        self._compute_controller_gains(controllerGains)

        TD_params = [] if TD_params is None else list(TD_params)
        if TD_method.lower() != "none":
            self.useTD = True
            self.TD_obj = TD(TD_method.lower(), self.dT, *TD_params)
            self.TD_obj.reset(0.0)
        else:
            self.useTD = False
            self.TD_obj = TD("euler", self.dT, 0.9)
        self.isInitialized = True

    def step(self, reference, output, *ref_derivatives):
        if not self.isInitialized:
            raise RuntimeError("Controller must be initialized before use")
        if self.useTD:
            self.TD_obj.update(reference)
            ref_filtered = self.TD_obj.y
            ref_derivs = [self.TD_obj.yd] + ([self.TD_obj.ydd] if self.n >= 2 else [])
        else:
            ref_filtered, ref_derivs = reference, []
        if ref_derivatives:
            ref_derivs = list(ref_derivatives[0]) if len(ref_derivatives) == 1 and hasattr(ref_derivatives[0], "__iter__") else list(ref_derivatives)

        u_for_eso = self.uHistory[0]
        for layer in range(self.numObservers):
            observer_input = output if layer == 0 else self.observerStates[0, layer - 1]
            previous_disturbance = np.zeros(self.controller_order) if layer == 0 else np.r_[np.zeros(self.n - 1), self.dT * np.sum(self.observerStates[-1, :layer]), 0.0]
            Ad = self.Ad if self.numObservers == 1 else self.Ad[:, :, layer]
            Beso = self.Beso if self.numObservers == 1 else self.Beso[:, layer]
            Ld = self.Ld if self.numObservers == 1 else self.Ld[:, layer]
            self.observerStates[:, layer] = (Ad @ self.observerStates[:, layer] +
                Beso * u_for_eso + Ld * observer_input + previous_disturbance)

        r_states = np.zeros(self.n)
        r_states[0] = ref_filtered
        m = min(self.n - 1, len(ref_derivs))
        if m:
            r_states[1:1 + m] = ref_derivs[:m]
        ref_nth = ref_derivs[self.n - 1] if len(ref_derivs) >= self.n else 0.0
        selected = self._selected_states()
        u = (np.dot(self.K, r_states - selected) + ref_nth - self.get_estimated_disturbance()) / self.b0
        u = self._saturate(u)
        self.uHistory[:-1] = self.uHistory[1:]
        self.uHistory[-1] = u
        self.uPrev = u
        return u

    def reset(self, XhatInit=None, uInit=0.0):
        if XhatInit is None:
            XhatInit = np.zeros(self.controller_order)
        XhatInit = np.asarray(XhatInit, dtype=float).reshape(-1)
        if len(XhatInit) != self.controller_order:
            raise ValueError(f"XhatInit must have length {self.controller_order}")
        self.observerStates = np.tile(XhatInit[:, None], (1, self.numObservers))
        self.uPrev = uInit
        self.uHistory = np.full(self.inputDelaySteps + 1, uInit)
        if self.useTD:
            self.TD_obj.reset(0.0)

    def set_td(self, method, *params):
        self.useTD = method.lower() != "none"
        if self.useTD:
            self.TD_obj = TD(method.lower(), self.dT, *params)
            self.TD_obj.reset(0.0)

    def set_saturation(self, u_min, u_max):
        self.uMin, self.uMax = u_min, u_max

    def set_input_delay(self, delay_sec):
        self.inputDelaySteps = max(0, int(round(delay_sec / self.dT)))
        self.uHistory = np.full(self.inputDelaySteps + 1, self.uPrev)

    def update_tuning(self, omega_c=None, omega_o=None, **legacy):
        if "Tsettle" in legacy:
            omega_c = 2.0 * (self.n + 1) / legacy["Tsettle"]
            omega_o = omega_c * legacy.get("kob", 10)
        if isinstance(omega_c, dict):
            omega_o = omega_c["kob"]
            omega_c = 2.0 * (self.n + 1) / omega_c["Tsettle"]
        self.omega_c = omega_c
        self.omega_o = np.full(self.numObservers, omega_o) if np.isscalar(omega_o) else np.asarray(omega_o)
        if len(self.omega_o) != self.numObservers:
            raise ValueError("omega_o must be scalar or contain one value per observer")
        self._compute_eso_gains()
        self._compute_controller_gains()

    def get_estimated_states(self):
        return np.r_[self._selected_states(), self.get_estimated_disturbance()]

    def get_observer_states(self, layer=None):
        if layer is None:
            return self.observerStates.copy()
        if layer < 1 or layer > self.numObservers:
            raise ValueError("Observer layer is out of range")
        return self.observerStates[:, layer - 1].copy()

    def get_estimated_disturbance(self):
        return float(np.sum(self.observerStates[-1, :]))

    def _compute_eso_gains(self, observer_gains=None):
        size = self.controller_order
        A = np.zeros((size, size))
        for i in range(self.n):
            A[i, i + 1] = 1.0
        B = np.zeros(size)
        B[self.n - 1] = self.b0
        C = np.zeros(size)
        C[0] = 1.0
        Ad_full = expm(A * self.dT)
        augmented = np.zeros((size + 1, size + 1))
        augmented[:size, :size], augmented[:size, size] = A, B
        self.Bd = expm(augmented * self.dT)[:size, size]
        self.Cd = C
        gains_matrix = np.zeros((size, self.numObservers))
        for layer, bandwidth in enumerate(self.omega_o):
            if observer_gains is None:
                desired = np.poly(np.full(size, np.exp(-bandwidth * self.dT)))
                dual = Ad_full.T
                dual_b = (C @ Ad_full).reshape(-1, 1)
                controllability = np.column_stack([np.linalg.matrix_power(dual, i) @ dual_b[:, 0] for i in range(size)])
                polynomial = np.linalg.matrix_power(dual, size)
                for coefficient in range(1, size + 1):
                    polynomial += desired[coefficient] * np.linalg.matrix_power(dual, size - coefficient)
                selector = np.zeros(size)
                selector[-1] = 1.0
                gains_matrix[:, layer] = polynomial.T @ np.linalg.solve(controllability.T, selector)
            else:
                gains = np.asarray(observer_gains, dtype=float)
                if gains.shape != (size, self.numObservers):
                    raise ValueError(f"observerGains must have shape {(size, self.numObservers)}")
                gains_matrix[:, layer] = gains[:, layer]
        ad = np.empty((size, size, self.numObservers))
        beso = np.empty((size, self.numObservers))
        for layer in range(self.numObservers):
            ad[:, :, layer] = Ad_full - np.outer(gains_matrix[:, layer], C @ Ad_full)
            beso[:, layer] = self.Bd - gains_matrix[:, layer] * (C @ self.Bd)
        self.Ld = gains_matrix[:, 0] if self.numObservers == 1 else gains_matrix
        self.Ad = ad[:, :, 0] if self.numObservers == 1 else ad
        self.Beso = beso[:, 0] if self.numObservers == 1 else beso

    def _compute_controller_gains(self, controller_gains=None):
        if controller_gains is not None:
            self.K = np.asarray(controller_gains, dtype=float).reshape(-1)
            if len(self.K) != self.n:
                raise ValueError(f"controllerGains must have length {self.n}")
            return
        self.K = np.array([math.comb(self.n, i) * self.omega_c ** i for i in range(self.n, 0, -1)])

    def _selected_states(self):
        return np.array([self.observerStates[i, self.estimateLayers[i] - 1] for i in range(self.n)])

    def _saturate(self, u):
        return min(u, self.uMax) if not np.isfinite(self.uMin) else max(u, self.uMin) if not np.isfinite(self.uMax) else np.clip(u, self.uMin, self.uMax)

    @property
    def use_cascaded(self):
        return self.useCascaded

    @property
    def use_td(self):
        return self.useTD

    @property
    def dt(self):
        return self.dT

    @property
    def input_delay_steps(self):
        return self.inputDelaySteps

    @property
    def u_history(self):
        return self.uHistory
