#include "adrc/ADRC.hpp"
#include "adrc/TD.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

// Matrix-vector multiplication helper
std::vector<double> mat_vec_mul(const std::vector<double>& A, const std::vector<double>& x, int rows, int cols) {
    std::vector<double> result(rows, 0.0);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            result[i] += A[i * cols + j] * x[j];
        }
    }
    return result;
}

// Row vector times column vector
double row_vec_mul(const std::vector<double>& C, const std::vector<double>& x) {
    double result = 0.0;
    for (size_t i = 0; i < x.size(); i++) {
        result += C[i] * x[i];
    }
    return result;
}

int main() {
    std::cout << "Cascaded vs Standard ADRC Comparison" << std::endl;
    
    // System: G(s) = 1.6 / (s^2 + 7.3s + 2.1)
    double b0 = 1.6;
    
    // Continuous state-space matrices
    // A = [0, 1; -2.1, -7.3], B = [0; 1.6], C = [1, 0]
    std::vector<double> A_cont = {0.0, 1.0, -2.1, -7.3};
    std::vector<double> B_cont = {0.0, 1.6};
    std::vector<double> C_cont = {1.0, 0.0};
    int n = 2;  // System order
    
    // Simulation parameters
    double dt = 0.001;
    double T = 15.0;
    int N = static_cast<int>(T / dt);
    
    // Discretize plant using matrix exponential approach
    // Augmented matrix for ZOH discretization
    std::vector<double> M(9, 0.0);  // 3x3 augmented matrix
    M[0] = -7.3; M[1] = -2.1; M[2] = 1.6;
    M[3] = 1.0;  M[4] = 0.0;  M[5] = 0.0;
    M[6] = 0.0;  M[7] = 0.0;  M[8] = 0.0;
    
        // Compute expm(M * dt)
    for (auto& val : M) val *= dt;
    std::vector<double> expM = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> term = expM;
    std::vector<double> Ad(4, 0.0);
    std::vector<double> Bd(4, 0.0);
    for (int k = 1; k <= 10; k++) {
        std::vector<double> temp(9, 0.0);
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                for (int k2 = 0; k2 < 3; k2++) {
                    temp[r * 3 + c] += term[r * 3 + k2] * M[k2 * 3 + c];
                }
            }
        }
        term = temp;
        
        double fact = 1.0;
        for (int i = 2; i <= k; i++) fact *= i;
        for (int i = 0; i < 9; i++) {
            expM[i] += term[i] / fact;
        }
    }
    
    Ad[0] = expM[0]; Ad[1] = expM[1];
    Ad[2] = expM[3]; Ad[3] = expM[4];
    Bd[0] = expM[2]; Bd[1] = expM[5];
    std::vector<double> Cd = C_cont;
    
    // Create Standard ADRC Controller
    ADRC ctrl_standard(n);
    ctrl_standard.initialize(1.0, 10.0, b0, -30.0, 30.0, 0.01, 
                             {}, 0.0, 1.0, 0.0, "none", {}, false);
    
    // Create Cascaded ADRC Controller
    ADRC ctrl_cascaded(n);
    ctrl_cascaded.initialize(1.0, 10.0, b0, -30.0, 30.0, 0.01,
                             {}, 0.0, 1.0, 0.0, "none", {}, true);
    
    // Generate Reference Signal
    double w = 2.0 * M_PI * 0.2;
    std::vector<double> ref(N);
    for (int k = 0; k < N; k++) {
        double t = (k + 1) * dt;
        ref[k] = std::sin(w * t);
    }
    
    // State vectors
    std::vector<double> x_std(n, 0.0);
    std::vector<double> x_cas(n, 0.0);
    
    // Storage
    std::vector<double> y_std(N, 0.0);
    std::vector<double> y_cas(N, 0.0);
    std::vector<double> u_std(N, 0.0);
    std::vector<double> u_cas(N, 0.0);
    std::vector<double> f_std(N, 0.0);
    std::vector<double> f_cas(N, 0.0);
    
    // Control update rate (10x slower than simulation)
    int ctrl_div = 10;
    
    for (int k = 0; k < N; k++) {
        // Standard ADRC
        double y_prev_std = (k > 0) ? y_std[k - 1] : 0.0;
        double y_prev_cas = (k > 0) ? y_cas[k - 1] : 0.0;
        
        if (k % ctrl_div == 0) {
            u_std[k] = ctrl_standard.step(ref[k], y_prev_std);
            u_cas[k] = ctrl_cascaded.step(ref[k], y_prev_cas);
        } else {
            int prev_idx = std::max(0, k - 1);
            u_std[k] = u_std[prev_idx];         // zero-order hold
            u_cas[k] = u_cas[prev_idx];         // zero-order hold
        }
        
        // Plant dynamics
        // x = Ad * x + Bd * u
        std::vector<double> temp_std(n, 0.0);
        temp_std[0] = Ad[0] * x_std[0] + Ad[1] * x_std[1] + Bd[0] * u_std[k];
        temp_std[1] = Ad[2] * x_std[0] + Ad[3] * x_std[1] + Bd[1] * u_std[k];
        x_std = temp_std;
        y_std[k] = Cd[0] * x_std[0] + Cd[1] * x_std[1];
        
        std::vector<double> temp_cas(n, 0.0);
        temp_cas[0] = Ad[0] * x_cas[0] + Ad[1] * x_cas[1] + Bd[0] * u_cas[k];
        temp_cas[1] = Ad[2] * x_cas[0] + Ad[3] * x_cas[1] + Bd[1] * u_cas[k];
        x_cas = temp_cas;
        y_cas[k] = Cd[0] * x_cas[0] + Cd[1] * x_cas[1];
        
        // Store disturbance estimates
        f_std[k] = ctrl_standard.getEstimatedDisturbance();
        f_cas[k] = ctrl_cascaded.getEstimatedDisturbance();
    }
    
    // Compute performance metrics
    double ITSE_std = 0.0, ITSE_cas = 0.0;
    double max_error_std = 0.0, max_error_cas = 0.0;
    double rms_error_std = 0.0, rms_error_cas = 0.0;
    double max_u_std = 0.0, max_u_cas = 0.0;
    
    for (int k = 0; k < N; k++) {
        double t = (k + 1) * dt;
        double error_std = ref[k] - y_std[k];
        double error_cas = ref[k] - y_cas[k];
        
        ITSE_std += t * error_std * error_std * dt;
        ITSE_cas += t * error_cas * error_cas * dt;
        
        max_error_std = std::max(max_error_std, std::abs(error_std));
        max_error_cas = std::max(max_error_cas, std::abs(error_cas));
        
        rms_error_std += error_std * error_std;
        rms_error_cas += error_cas * error_cas;
        
        max_u_std = std::max(max_u_std, std::abs(u_std[k]));
        max_u_cas = std::max(max_u_cas, std::abs(u_cas[k]));
    }
    
    rms_error_std = std::sqrt(rms_error_std / N);
    rms_error_cas = std::sqrt(rms_error_cas / N);
    
    // Print Performance Metrics
    std::cout << "\n=== Performance Comparison ===" << std::endl;
    std::cout << "Standard ADRC:" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  ITSE: " << ITSE_std << std::endl;
    std::cout << "  Max |error|: " << max_error_std << std::endl;
    std::cout << "  RMS error: " << rms_error_std << std::endl;
    std::cout << "  Max |control|: " << max_u_std << std::endl;
    std::cout << std::endl;
    
    std::cout << "Cascaded ADRC:" << std::endl;
    std::cout << "  ITSE: " << ITSE_cas << std::endl;
    std::cout << "  Max |error|: " << max_error_cas << std::endl;
    std::cout << "  RMS error: " << rms_error_cas << std::endl;
    std::cout << "  Max |control|: " << max_u_cas << std::endl;
    
    std::cout << "\nExample completed successfully!" << std::endl;
    
    return 0;
}