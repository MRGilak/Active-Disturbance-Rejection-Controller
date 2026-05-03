# Code Documentation

Below, the MATLAB implementation has been explained. The Python implementation is very similar and hence its explanation has been skipped here. 

## TD Class

**File:** [TD.m](TD.m)

Object-oriented implementation of Tracking Differentiator with multiple methods.

### Properties

**Private:**
- `method` (char): TD method ('euler', 'tod', 'ld', 'red', 'intd')
- `dT` (double): Sample time
- `params` (cell): Method-specific parameters
- `state` (struct): Internal state variables

**Public (Read-only):**
- `y` (double): Filtered output
- `yd` (double): First derivative estimate
- `ydd` (double): Second derivative estimate

### Constructor

```matlab
obj = TD(method, dT, varargin)
```

**Parameters:**
- `method`: String specifying TD method
- `dT`: Sample time in seconds
- `varargin`: Method-specific tuning parameters
  - `'euler'`: a (filter coefficient, default 0.9)
  - `'tod'`: r (convergence rate, default 1)
  - `'ld'`: lambda (bandwidth, default 1)
  - `'red'`: lambda1, lambda2 (defaults 1, 1)
  - `'intd'`: alpha, beta, gamma, r (defaults 1, 1, 1, 1)

**Example:**
```matlab
td = TD('tod', 0.01, 20);  % TOD with r=20, dT=0.01s
```

### Methods

**`update(ref)`**

Update TD with new reference value.

**Parameters:**
- `ref`: Current reference signal value

**Updates:** `y`, `yd`, `ydd` properties

**Example:**
```matlab
td.update(1.0);
filtered_ref = td.y;
ref_dot = td.yd;
```

**`reset(initialValue)`**

Reset TD to initial state.

**Parameters:**
- `initialValue`: Initial value (default: 0)

**`setParameters(varargin)`**

Update tuning parameters without recreating object.

**`setSampleTime(dT)`**

Update sample time.

---

## ADRC Class

**File:** [ADRC.m](ADRC.m)

Comprehensive ADRC controller with ESO, optional TD, delay compensation, and saturation.

### Properties

**Private:**
- `n` (double): System order (1-4)
- `dT` (double): Sample time
- `Ld`, `K` (double): Observer and controller gains
- `Ad`, `Bd`, `Cd` (double): Discrete ESO matrices
- `b0` (double): Control gain estimate
- `Tsettle`, `kob` (double): Tuning parameters
- `Ke` (double): Error scaling gain
- `Xhat` (double): Extended state estimates [x₁...xₙ, f]
- `uPrev` (double): Previous control input
- `uMin`, `uMax` (double): Saturation limits
- `inputDelaySteps` (double): Delay in controller steps
- `uHistory` (double): Control history buffer
- `useTD` (logical): TD enable flag
- `TD_obj` (TD): TD object instance

**Public (Read-only):**
- `isInitialized` (logical): Initialization status
- `controllerOrder` (double): n+1 (extended state dimension)

### Constructor

```matlab
obj = ADRC(systemOrder)
```

**Parameters:**
- `systemOrder`: System order (1-4)

**Example:**
```matlab
controller = ADRC(2);  % For 2nd-order system
```

### Methods

**`initialize(varargin)`**

Initialize controller with name-value pairs.

**Name-Value Parameters:**
- `'Tsettle'`: Settling time (default: 1.0)
- `'kob'`: Observer bandwidth multiplier (default: 10)
- `'b0'`: Control gain estimate (default: 1.0)
- `'uMin'`: Lower saturation limit (default: -inf)
- `'uMax'`: Upper saturation limit (default: inf)
- `'dT'`: Sample time (default: 0.01)
- `'XhatInit'`: Initial extended state (default: zeros)
- `'uInit'`: Initial control input (default: 0)
- `'Ke'`: Error scaling gain (default: 1)
- `'inputDelay'`: Input delay in seconds (default: 0)
- `'TD_method'`: TD method or 'none' (default: 'none')
- `'TD_params'`: Cell array of TD parameters (default: {})

**Example:**
```matlab
controller.initialize('Tsettle', 1.0, 'kob', 10, 'b0', 1.6, ...
                     'dT', 0.01, 'uMin', -10, 'uMax', 10, ...
                     'inputDelay', 0.05, ...
                     'TD_method', 'tod', 'TD_params', {20});
```

**`u = step(reference, output, varargin)`**

Execute one control step.

**Parameters:**
- `reference`: Reference signal value
- `output`: System output (measurement)
- `varargin`: Optional reference derivatives [r', r'', ...] (vector or individual arguments)

**Returns:**
- `u`: Control signal

**Example:**
```matlab
u = controller.step(ref, y);  % Without derivatives
u = controller.step(ref, y, ref_dot, ref_dotdot);  % With derivatives
```

**`reset(XhatInit, uInit)`**

Reset controller state.

**Parameters:**
- `XhatInit`: Initial extended state (optional)
- `uInit`: Initial control input (optional)

**`setTD(method, varargin)`**

Configure or update tracking differentiator.

**Parameters:**
- `method`: TD method or 'none'
- `varargin`: TD parameters

**Example:**
```matlab
controller.setTD('tod', 15);  % Enable TOD with r=15
controller.setTD('none');     % Disable TD
```

**`setSaturation(uMin, uMax)`**

Update saturation limits.

**`setInputDelay(delaySec)`**

Update input delay setting.

**`updateTuning(Tsettle, kob)`**

Update controller tuning parameters and recompute gains.

**`Xhat = getEstimatedStates()`**

Get current extended state estimates [x₁, ..., xₙ, f]ᵀ.

**`f = getEstimatedDisturbance()`**

Get estimated total disturbance (last element of Xhat).

---

# Scripts

## ADRC_app.m

Interactive GUI application for ADRC simulation.

**Features:**
- System transfer function input (numerator/denominator)
- Controller tuning sliders (Tsettle, kob, b0)
- Input delay configuration (matched/unmatched)
- Control saturation limits
- Reference signal generator (Step, Sinusoid, Sawtooth)
- TD method selection with parameter adjustment
- Real-time plotting

**Usage:**
```matlab
ADRC_app
```

## demo.m

Comprehensive demonstration script with three examples:

1. **Basic ADRC**: Step response without TD
2. **ADRC with TD**: Sinusoidal tracking with TOD
3. **Input Delay Compensation**: Step response with input delay