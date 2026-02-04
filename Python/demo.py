import numpy as np
import matplotlib.pyplot as plt
import control
from ADRC import ADRC

def example_1_basic_adrc():
    print('Example 1: Basic ADRC without TD')
    
    # System: G(s) = 1.6 / (s^2 + 7.3s + 2.1)
    num = [1.6]
    den = [1, 7.3, 2.1]
    
    # Create transfer function and convert to state space
    sys = control.tf(num, den)
    sys_ss = control.tf2ss(sys)
    n = sys_ss.nstates
    
    # Simulation parameters
    dt = 0.001
    T = 10
    N = int(T / dt)
    time = np.arange(dt, T + dt, dt)[:N]
    
    # Discretize plant
    sys_d = sys_ss.sample(dt, method='zoh')
    Ad = sys_d.A
    Bd = sys_d.B
    Cd = sys_d.C
    
    # Create and initialize ADRC controller
    controller = ADRC(n)
    controller.initialize(Tsettle=1.0, kob=10, b0=1.6, dt=0.01, 
                         u_min=-10, u_max=10)
    
    # Generate step reference
    ref = np.zeros(N)
    ref[99:] = 1.0
    
    # Simulate
    x = np.zeros(n)
    y = np.zeros(N)
    u = np.zeros(N)
    xhat = np.zeros((N, n + 1))
    
    for k in range(N):
        # Controller update (at slower rate)
        if k % 10 == 0:
            u[k] = controller.step(ref[k], y[max(0, k - 1)])
        else:
            u[k] = u[k - 1]
        
        # Plant simulation
        x = Ad @ x + (Bd * u[k]).flatten()
        y[k] = (Cd @ x).item()
        
        # Store estimates
        xhat[k, :] = controller.get_estimated_states()
    
    # Plot results
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle('Example 1: Basic ADRC')
    
    ax1.plot(time, y, 'b', linewidth=1.5, label='Output y')
    ax1.plot(time, ref, 'r--', linewidth=1.2, label='Reference r')
    ax1.plot(time, xhat[:, 0], 'g', linewidth=1.2, label=r'Estimate $\hat{y}$')
    ax1.grid(True)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Output')
    ax1.legend(loc='best')
    ax1.set_title('System Response')
    
    ax2.plot(time, u, 'b', linewidth=1.5)
    ax2.grid(True)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Control Input')
    ax2.set_title('Control Signal')
    
    plt.tight_layout()
    plt.show()


def example_2_adrc_with_td():
    print('\nExample 2: ADRC with TD for sinusoidal reference')
    
    # System: G(s) = 1.6 / (s^2 + 7.3s + 2.1)
    num = [1.6]
    den = [1, 7.3, 2.1]
    
    # Create transfer function and convert to state space
    sys = control.tf(num, den)
    sys_ss = control.tf2ss(sys)
    n = sys_ss.nstates
    
    # Simulation parameters
    dt = 0.001
    T = 10
    N = int(T / dt)
    time = np.arange(dt, T + dt, dt)[:N]
    
    # Discretize plant
    sys_d = sys_ss.sample(dt, method='zoh')
    Ad = sys_d.A
    Bd = sys_d.B
    Cd = sys_d.C
    
    # Reset controller with TD
    controller2 = ADRC(n)
    controller2.initialize(Tsettle=1.0, kob=10, b0=1.6, dt=0.01,
                          u_min=-30, u_max=30,
                          TD_method='euler', TD_params=[0.6])
    
    # Generate sinusoidal reference
    w = 2 * np.pi * 0.5
    ref_sin = np.sin(w * time)
    
    # Simulate
    x2 = np.zeros(n)
    y2 = np.zeros(N)
    u2 = np.zeros(N)
    xhat2 = np.zeros((N, n + 1))
    
    for k in range(N):
        # Controller update
        if k % 10 == 0:
            u2[k] = controller2.step(ref_sin[k], y2[max(0, k - 1)])
        else:
            u2[k] = u2[k - 1]
        
        # Plant simulation
        x2 = Ad @ x2 + (Bd * u2[k]).flatten()
        y2[k] = (Cd @ x2).item()
        
        # Store estimates
        xhat2[k, :] = controller2.get_estimated_states()
    
    # Plot results
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle('Example 2: ADRC with TD')
    
    ax1.plot(time, y2, 'b', linewidth=1.5, label='Output y')
    ax1.plot(time, ref_sin, 'r--', linewidth=1.2, label='Reference r')
    ax1.plot(time, xhat2[:, 0], 'g', linewidth=1.2, label=r'Estimate $\hat{y}$')
    ax1.grid(True)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Output')
    ax1.legend(loc='best')
    ax1.set_title('System Response with TD (Sinusoidal Reference)')
    
    ax2.plot(time, u2, 'b', linewidth=1.5)
    ax2.grid(True)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Control Input')
    ax2.set_title('Control Signal')
    
    plt.tight_layout()
    plt.show()


def example_3_adrc_with_delay():
    print('\nExample 3: ADRC with input delay compensation')
    
    # System: G(s) = 1.6 / (s^2 + 7.3s + 2.1)
    num = [1.6]
    den = [1, 7.3, 2.1]
    
    # Create transfer function and convert to state space
    sys = control.tf(num, den)
    sys_ss = control.tf2ss(sys)
    n = sys_ss.nstates
    
    # Simulation parameters
    dt = 0.001
    T = 10
    N = int(T / dt)
    time = np.arange(dt, T + dt, dt)[:N]
    
    # Discretize plant
    sys_d = sys_ss.sample(dt, method='zoh')
    Ad = sys_d.A
    Bd = sys_d.B
    Cd = sys_d.C
    
    # Reset controller with input delay
    input_delay = 0.2  # 200ms delay
    controller3 = ADRC(n)
    controller3.initialize(Tsettle=1.0, kob=10, b0=1.6, dt=0.01,
                          u_min=-10, u_max=10,
                          input_delay=input_delay)
    
    # Generate step reference
    ref_step = np.zeros(N)
    ref_step[999:] = 1.5
    
    # Setup plant delay buffer
    delay_samples = int(round(input_delay / dt))
    u_buffer = np.zeros(delay_samples + 1)
    
    # Simulate
    x3 = np.zeros(n)
    y3 = np.zeros(N)
    u3 = np.zeros(N)
    u_delayed = np.zeros(N)
    xhat3 = np.zeros((N, n + 1))
    
    for k in range(N):
        # Controller update
        if k % 10 == 0:
            u3[k] = controller3.step(ref_step[k], y3[max(0, k - 1)])
        else:
            u3[k] = u3[k - 1]
        
        # Apply delay to plant input
        u_delayed[k] = u_buffer[0]
        u_buffer[:-1] = u_buffer[1:]
        u_buffer[-1] = u3[k]
        
        # Plant simulation with delayed input
        x3 = Ad @ x3 + (Bd * u_delayed[k]).flatten()
        y3[k] = (Cd @ x3).item()
        
        # Store estimates
        xhat3[k, :] = controller3.get_estimated_states()
    
    # Plot results
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle('Example 3: ADRC with Input Delay')
    
    ax1.plot(time, y3, 'b', linewidth=1.5, label='Output y')
    ax1.plot(time, ref_step, 'r--', linewidth=1.2, label='Reference r')
    ax1.plot(time, xhat3[:, 0], 'g', linewidth=1.2, label=r'Estimate $\hat{y}$')
    ax1.grid(True)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Output')
    ax1.legend(loc='best')
    ax1.set_title(f'System Response with {int(input_delay*1000)}ms Input Delay')
    
    ax2.plot(time, u3, 'b', linewidth=1.5, label=r'Controller $u(t)$')
    ax2.plot(time, u_delayed, 'r--', linewidth=1.2, label=r'Applied $u(t-\tau)$')
    ax2.grid(True)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Control Input')
    ax2.legend(loc='best')
    ax2.set_title('Control Signal')
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # Run all examples
    example_1_basic_adrc()
    example_2_adrc_with_td()
    example_3_adrc_with_delay()
    
    print('\nAll examples completed successfully!')
