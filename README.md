# Active Disturbance Rejection Control (ADRC)

MATLAB, Simulink and Python implementation of Active Disturbance Rejection Control with Extended State Observer, Tracking Differentiator, input delay compensation, and control saturation.

---

## Theoretical Background

### System Representation

Consider an nth-order SISO system:

$$y^{(n)} = f(y, \dot{y}, \ldots, y^{(n-1)}, w(t)) + b_0 u$$

where:
- $y$ is the system output
- $u$ is the control input
- $b_0$ is the control gain (nominal)
- $f(\cdot)$ represents the total disturbance (internal dynamics + external disturbances)
- $w(t)$ represents external disturbances

### State-Space Form

Define state variables: 
$$x_1 = y, \quad x_2 = \dot{y}, \quad \ldots, \quad x_n = y^{(n-1)}$$

The system can be written as:

$$\begin{aligned}
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= x_3 \\
&\vdots \\
\dot{x}_n &= f + b_0 u \\
y &= x_1
\end{aligned}$$

### Extended State Observer (ESO)

To estimate both the states and the total disturbance $f$, we augment the state vector with $x_{n+1} = f$ (assuming $\dot{f} \approx 0$):

$$\begin{aligned}
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= x_3 \\
&\vdots \\
\dot{x}_n &= x_{n+1} + b_0 u \\
\dot{x}_{n+1} &= \dot{f} \approx 0 \\
y &= x_1
\end{aligned}$$

In matrix form:

$$\begin{aligned}
\dot{\mathbf{x}} &= \mathbf{A} \mathbf{x} + \mathbf{B} u \\
y &= \mathbf{C} \mathbf{x}
\end{aligned}$$

where:

$$\mathbf{A} = \begin{bmatrix}
0 & 1 & 0 & \cdots & 0 & 0 \\
0 & 0 & 1 & \cdots & 0 & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots & \vdots \\
0 & 0 & 0 & \cdots & 0 & 1 \\
0 & 0 & 0 & \cdots & 0 & 0
\end{bmatrix}_{(n+1) \times (n+1)}, \quad
\mathbf{B} = \begin{bmatrix}
0 \\ 0 \\ \vdots \\ b_0 \\ 0
\end{bmatrix}, \quad
\mathbf{C} = \begin{bmatrix}
1 & 0 & \cdots & 0 & 0
\end{bmatrix}$$

The ESO is designed as:

$$\dot{\hat{\mathbf{x}}} = \mathbf{A} \hat{\mathbf{x}} + \mathbf{B} u + \mathbf{L}(y - \hat{y})$$

where $\mathbf{L} = [l_1, l_2, \ldots, l_{n+1}]^T$ is the observer gain vector.

### Observer Gain Selection

The observer gains are selected using bandwidth parameterization. We place all observer poles at $s = -\omega_o$ where $\omega_o$ is the observer bandwidth:

$$\det(sI - (\mathbf{A} - \mathbf{L}\mathbf{C})) = (s + \omega_o)^{n+1}$$

For different system orders:

**First-order system (n=1):**
$$\begin{aligned}
l_1 &= 2\omega_o \\
l_2 &= \omega_o^2
\end{aligned}$$

**Second-order system (n=2):**
$$\begin{aligned}
l_1 &= 3\omega_o \\
l_2 &= 3\omega_o^2 \\
l_3 &= \omega_o^3
\end{aligned}$$

**Third-order system (n=3):**
$$\begin{aligned}
l_1 &= 4\omega_o \\
l_2 &= 6\omega_o^2 \\
l_3 &= 4\omega_o^3 \\
l_4 &= \omega_o^4
\end{aligned}$$

**Fourth-order system (n=4):**
$$\begin{aligned}
l_1 &= 5\omega_o \\
l_2 &= 10\omega_o^2 \\
l_3 &= 10\omega_o^3 \\
l_4 &= 5\omega_o^4 \\
l_5 &= \omega_o^5
\end{aligned}$$

The observer bandwidth is typically chosen as:
$$\omega_o = k_{ob} \cdot \omega_c$$

where $\omega_c = -s_{cl}$ and $s_{cl} = \frac{-4}{T_{settle}}$ is the desired closed-loop pole location, and $k_{ob}$ is a multiplier (typically 5-20).

### Discrete-Time ESO

The continuous ESO is discretized using exact discretization:

$$\mathbf{A}_d = e^{\mathbf{A} \Delta T}$$

$$\mathbf{B}_d = \int_0^{\Delta T} e^{\mathbf{A} \tau} d\tau \cdot \mathbf{B} = \mathbf{A}^{-1}(\mathbf{A}_d - \mathbf{I})\mathbf{B}$$

For numerical stability, we compute:

$$\mathbf{B}_d = \sum_{i=1}^{n} \frac{(\Delta T)^i}{i!} \mathbf{A}^{i-1} \mathbf{B}$$

The discrete observer gains are computed using pole placement in the z-domain. For a pole at $z = e^{s \Delta T}$ where $s = -\omega_o$:

$$z_{eso} = e^{-\omega_o \Delta T}$$

The discrete gains $L_d$ are selected such that the characteristic polynomial matches $(z - z_{eso})^{n+1}$.

**For n=1:**
$$\begin{aligned}
L_{d,1} &= 1 - z_{eso}^2 \\
L_{d,2} &= \frac{(1 - z_{eso})^2}{\Delta T}
\end{aligned}$$

**For n=2:**
$$\begin{aligned}
L_{d,1} &= 1 - z_{eso}^3 \\
L_{d,2} &= \frac{3(1 + z_{eso})(1 - z_{eso})^2}{2\Delta T} \\
L_{d,3} &= \frac{(1 - z_{eso})^3}{\Delta T^2}
\end{aligned}$$

**For n=3:**
$$\begin{aligned}
L_{d,1} &= 1 - z_{eso}^4 \\
L_{d,2} &= \frac{(1 - z_{eso})^2(11 + z_{eso}(14 + 11z_{eso}))}{6\Delta T} \\
L_{d,3} &= \frac{2(1 - z_{eso})^3(1 + z_{eso})}{\Delta T^2} \\
L_{d,4} &= \frac{(1 - z_{eso})^4}{\Delta T^3}
\end{aligned}$$

**For n=4:**
$$\begin{aligned}
L_{d,1} &= 1 - z_{eso}^5 \\
L_{d,2} &= \frac{5(1 - z_{eso})^2(1 + z_{eso})(5 + z_{eso}(2 + 5z_{eso}))}{12\Delta T} \\
L_{d,3} &= \frac{5(1 - z_{eso})^3(7 + z_{eso}(10 + 7z_{eso}))}{12\Delta T^2} \\
L_{d,4} &= \frac{5(1 - z_{eso})^4(1 + z_{eso})}{2\Delta T^3} \\
L_{d,5} &= \frac{(1 - z_{eso})^5}{\Delta T^4}
\end{aligned}$$

The discrete ESO update equation:

$$\hat{\mathbf{x}}[k] = \mathbf{A}_d \hat{\mathbf{x}}[k-1] + \mathbf{B}_d u[k-1] + \mathbf{L}_d (y[k] - \mathbf{C}_d \mathbf{A}_d \hat{\mathbf{x}}[k-1])$$

Simplified form:

$$\hat{\mathbf{x}}[k] = (\mathbf{A}_d - \mathbf{L}_d \mathbf{C}_d \mathbf{A}_d) \hat{\mathbf{x}}[k-1] + (\mathbf{B}_d - \mathbf{L}_d \mathbf{C}_d \mathbf{B}_d) u[k-1] + \mathbf{L}_d y[k]$$

### Control Law

The control law compensates for the estimated disturbance and tracks the reference:

$$u_0 = \frac{1}{b_0} \left[ r^{(n)} - \sum_{i=1}^{n} k_i (r^{(i-1)} - \hat{x}_i) - \hat{x}_{n+1} \right]$$

where $r$ is the reference signal and $k_i$ are the state feedback gains.

Equivalently:

$$u_0 = \frac{1}{b_0} \left[ \mathbf{K}^T (\mathbf{r} - \hat{\mathbf{x}}) \right]$$

where:

$$\mathbf{r} = \begin{bmatrix} r \\ r^{(1)} \\ \vdots \\ r^{(n-1)} \\ r^{(n)} \end{bmatrix}, \quad
\mathbf{K} = \begin{bmatrix} k_1 \\ k_2 \\ \vdots \\ k_n \\ 1 \end{bmatrix}$$

### Controller Gain Selection

The controller gains are selected by pole placement. All closed-loop poles are placed at $s = -\omega_c$:

$$\det(sI - (\mathbf{A} - \mathbf{B}\mathbf{K}^T)) = (s + \omega_c)^n$$

where $\omega_c = \frac{-4}{T_{settle}}$ for a desired settling time $T_{settle}$.

For an nth-order system:

$$k_i = \binom{n}{i} \omega_c^i, \quad i = 1, 2, \ldots, n$$

**Examples:**

For n=2: $k_1 = 2\omega_c$, $k_2 = \omega_c^2$

For n=3: $k_1 = 3\omega_c$, $k_2 = 3\omega_c^2$, $k_3 = \omega_c^3$

### Tracking Differentiator (TD)

When reference derivatives are not available, a Tracking Differentiator is used to estimate them from the reference signal. Five TD methods are implemented:

#### 1. Euler-based TD
Low-pass filter with numerical differentiation:

$$\begin{aligned}
y_f[k] &= a \cdot y_f[k-1] + (1-a) \cdot r[k] \\
\dot{y}_f[k] &= \frac{y_f[k] - y_f[k-1]}{\Delta T}
\end{aligned}$$

Parameter: $a \in (0,1)$ (filter coefficient)

#### 2. Time-Optimal Differentiator (TOD)
$$\begin{aligned}
\dot{v}_1 &= v_2 \\
\dot{v}_2 &= -r \cdot \text{sign}\left( v_1 - r + \frac{v_2 |v_2|}{2r} \right)
\end{aligned}$$

Parameter: $r > 0$ (convergence rate)

#### 3. Linear Differentiator (LD)
$$\begin{aligned}
\dot{v}_1 &= v_2 - \frac{v_1 - r}{\lambda} \\
\dot{v}_2 &= -\frac{2(v_1 - r)}{\lambda}
\end{aligned}$$

Parameter: $\lambda > 0$ (bandwidth)

#### 4. Robust Exact Differentiator (RED)
$$\begin{aligned}
\dot{v}_1 &= v_2 - \lambda_1 |e|^{0.5} \text{sign}(e) \\
\dot{v}_2 &= -\lambda_2 \text{sign}(e)
\end{aligned}$$

where $e = v_1 - r$

Parameters: $\lambda_1, \lambda_2 > 0$

#### 5. Improved Nonlinear TD (INTD)
$$\begin{aligned}
\dot{v}_1 &= v_2 \\
\dot{v}_2 &= -r^2 \tanh\left( \frac{\beta v_1 - (1-\alpha) r}{\gamma} \right) - r v_2
\end{aligned}$$

Parameters: $\alpha \in [0,1]$, $\beta, \gamma, r > 0$

All TDs are integrated using forward Euler with sample time $\Delta T$.

### Input Delay Compensation

When the system has input delay $\tau$, the actual plant input is $u(t-\tau)$. The ESO is modified to use the delayed control signal:

$$\hat{\mathbf{x}}[k] = \mathbf{A}_d \hat{\mathbf{x}}[k-1] + \mathbf{B}_d u[k-1-d] + \mathbf{L}_d y[k]$$

where $d = \lfloor \tau / \Delta T \rfloor$ is the delay in samples.

The controller computes the current control signal, while the ESO uses the control signal from $d$ steps ago, which is the signal currently affecting the plant.

### Control Saturation

When control saturation limits $[u_{min}, u_{max}]$ exist, the control signal is saturated:

$$u = \begin{cases}
u_{max} & \text{if } u_0 > u_{max} \\
u_{min} & \text{if } u_0 < u_{min} \\
u_0 & \text{otherwise}
\end{cases}$$

The saturated control signal $u$ is fed back to the ESO to maintain consistency between the ESO's prediction and the actual plant input.

---

## Code Documentation

Below, the MATLAB implementation has been explained. The Python implementation is very similar and hence its explanation has been skipped here.

### TD Class

**File:** `TD.m`

Object-oriented implementation of Tracking Differentiator with multiple methods.

#### Properties

**Private:**
- `method` (char): TD method ('euler', 'tod', 'ld', 'red', 'intd')
- `dT` (double): Sample time
- `params` (cell): Method-specific parameters
- `state` (struct): Internal state variables

**Public (Read-only):**
- `y` (double): Filtered output
- `yd` (double): First derivative estimate
- `ydd` (double): Second derivative estimate

#### Constructor

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

#### Methods

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

### ADRC Class

**File:** `ADRC.m`

Comprehensive ADRC controller with ESO, optional TD, delay compensation, and saturation.

#### Properties

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

#### Constructor

```matlab
obj = ADRC(systemOrder)
```

**Parameters:**
- `systemOrder`: System order (1-4)

**Example:**
```matlab
controller = ADRC(2);  % For 2nd-order system
```

#### Methods

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

## Scripts

### ADRC_app.m

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

### demo.m

Comprehensive demonstration script with three examples:

1. **Basic ADRC**: Step response without TD
2. **ADRC with TD**: Sinusoidal tracking with TOD
3. **Input Delay Compensation**: Step response with input delay

---

## Implementation Notes

1. **Sample Time**: Controller sample time `dT` can differ from simulation time step `dt`. The controller should be called at rate `dT`.

2. **Delay Compensation**: Input delay is specified in seconds and internally converted to discrete steps. Delay buffer maintains control history.

3. **Initialization**: Always call `initialize()` before `step()`. The controller will throw an error if used uninitialized.

4. **TD Integration**: When TD is enabled, it is automatically updated within `step()`. Manual reference derivatives can still be provided via `varargin` to override TD estimates.

5. **State Estimation**: Access estimated states via `getEstimatedStates()` for monitoring or additional processing.

6. **Tuning Guidelines**:
   - Start with `Tsettle` = desired settling time
   - Set `kob` = 5-20 (higher = faster observer, more noise sensitivity)
   - Ensure `b0` matches nominal plant gain
   - Adjust `Ke` if using non-unity error scaling

---

## References

1. Han, J. (2009). "From PID to Active Disturbance Rejection Control". IEEE Transactions on Industrial Electronics.

2. Gao, Z. (2006). "Active Disturbance Rejection Control: A Paradigm Shift in Feedback Control System Design". American Control Conference.

3. Herbst, G. (2013). "A Simulative Study on Active Disturbance Rejection Control (ADRC) as a Control Tool for Practitioners". Electronics.

4. Zheng, Q., Gao, Z. (2010). "On Practical Applications of Active Disturbance Rejection Control". Chinese Control Conference.

5. Madoński, R., & Herman, P. (2015). Survey on methods of increasing the efficiency of extended state disturbance observers. ISA transactions, 56, 18-27.

6. Madoński, R., & Herman, P. (2015). Survey on methods of increasing the efficiency of extended state disturbance observers. ISA transactions, 56, 18-27.
