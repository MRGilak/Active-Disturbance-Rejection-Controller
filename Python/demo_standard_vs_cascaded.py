import numpy as np
import matplotlib.pyplot as plt
import control
from ADRC import ADRC

def example_standard_vs_cascaded():
    print('Cascaded vs Standard ADRC Comparison')
    
    # System: G(s) = 1.6 / (s^2 + 7.3s + 2.1)
    num = [1.6]
    den = [1, 7.3, 2.1]
    b0 = 1.6
    
    # Create transfer function and convert to state space
    sys = control.tf(num, den)
    sys_ss = control.tf2ss(sys)
    n = sys_ss.nstates
    
    # Simulation parameters
    dt = 0.001
    T = 15
    N = int(T / dt)
    time = np.arange(dt, T + dt, dt)[:N]
    
    # Discretize plant
    sys_d = sys_ss.sample(dt, method='zoh')
    Ad = sys_d.A
    Bd = sys_d.B
    Cd = sys_d.C
    
    # Create Standard ADRC Controller
    ctrl_standard = ADRC(n)
    ctrl_standard.initialize(Tsettle=1.0, kob=10, b0=b0, dt=0.01,
                             u_min=-30, u_max=30, Ke=1.0, use_cascaded=False)
    
    # Create Cascaded ADRC Controller
    ctrl_cascaded = ADRC(n)
    ctrl_cascaded.initialize(Tsettle=1.0, kob=10, b0=b0, dt=0.01,
                             u_min=-30, u_max=30, Ke=1.0, use_cascaded=True)
    
    # Generate Reference Signal
    w = 2 * np.pi * 0.2
    ref = np.sin(w * time)
    
    # State vectors
    x_std = np.zeros(n)
    x_cas = np.zeros(n)
    
    # Storage
    y_std = np.zeros(N)
    y_cas = np.zeros(N)
    u_std = np.zeros(N)
    u_cas = np.zeros(N)
    xhat_std = np.zeros((N, n + 1))
    zhat_cas = np.zeros((N, n + 1))
    mhat_cas = np.zeros((N, n + 1))
    f_std = np.zeros(N)
    f_cas = np.zeros(N)
    
    # Control update rate (10x slower than simulation)
    ctrl_div = 10
    
    for k in range(N):
        # Standard ADRC
        if k % ctrl_div == 0:
            u_std[k] = ctrl_standard.step(ref[k], y_std[max(0, k - 1)])
            u_cas[k] = ctrl_cascaded.step(ref[k], y_cas[max(0, k - 1)])
        else:
            u_std[k] = u_std[max(0, k - 1)]
            u_cas[k] = u_cas[max(0, k - 1)]
        
        # Plant dynamics
        x_std = Ad @ x_std + (Bd * u_std[k]).flatten()
        y_std[k] = (Cd @ x_std).item()
        
        x_cas = Ad @ x_cas + (Bd * u_cas[k]).flatten()
        y_cas[k] = (Cd @ x_cas).item()
        
        # Store estimates
        xhat_std[k, :] = ctrl_standard.get_estimated_states()
        zhat_cas[k, :] = ctrl_cascaded.get_z_states()
        mhat_cas[k, :] = ctrl_cascaded.get_m_states()
        
        # Store disturbance estimates
        f_std[k] = ctrl_standard.get_estimated_disturbance()
        f_cas[k] = ctrl_cascaded.get_estimated_disturbance()
    
    # Plot Results
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('Cascaded vs Standard ADRC', fontsize=14)
    
    # Output tracking
    ax1.plot(time, y_std, 'b', linewidth=1.2, label='Standard')
    ax1.plot(time, y_cas, 'r', linewidth=1.2, label='Cascaded')
    ax1.plot(time, ref, 'k--', linewidth=1, label='Reference')
    ax1.grid(True)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Output')
    ax1.legend(loc='best')
    ax1.set_title('Output Tracking')
    
    # Tracking error
    ax2.plot(time, ref - y_std, 'b', linewidth=1.2, label='Standard')
    ax2.plot(time, ref - y_cas, 'r', linewidth=1.2, label='Cascaded')
    ax2.grid(True)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error')
    ax2.legend(loc='best')
    ax2.set_title('Tracking Error')
    
    # Control signal
    ax3.plot(time, u_std, 'b', linewidth=1.2, label='Standard')
    ax3.plot(time, u_cas, 'r', linewidth=1.2, label='Cascaded')
    ax3.grid(True)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Control')
    ax3.legend(loc='best')
    ax3.set_title('Control Input')
    
    # Disturbance estimation
    ax4.plot(time, f_std, 'b', linewidth=1.2, label='$f_{std}$')
    ax4.plot(time, f_cas, 'r', linewidth=1.2, label='$f_{cas}$')
    ax4.plot(time, zhat_cas[:, -1], 'r--', linewidth=1, label='$z_{n+1}$')
    ax4.plot(time, mhat_cas[:, -1], 'g--', linewidth=1, label='$m_{n+1}$')
    ax4.grid(True)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Disturbance')
    ax4.legend(loc='best')
    ax4.set_title('Disturbance Estimation')
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    example_standard_vs_cascaded()
    print('\nExample completed successfully!')