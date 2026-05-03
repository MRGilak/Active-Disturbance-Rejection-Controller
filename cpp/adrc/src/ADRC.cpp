#include "adrc/ADRC.hpp"
#include "adrc/TD.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

// Constructor
ADRC::ADRC(int systemOrder) 
    : n(systemOrder), 
      dT(0.01),
      NADRC(systemOrder + 1),
      b0(1.0),
      Tsettle(1.0),
      kob(10.0),
      Ke(1.0),
      uPrev(0.0),
      useCascaded(false),
      uMin(-std::numeric_limits<double>::infinity()),
      uMax(std::numeric_limits<double>::infinity()),
      inputDelaySteps(0),
      useTD(false),
      TD_obj(nullptr),
      isInitialized(false) {
    
    if (systemOrder < 1 || systemOrder > 4) {
        throw std::invalid_argument("System order must be between 1 and 4");
    }
    
    xhat.resize(NADRC, 0.0);
    mhat.resize(NADRC, 0.0);
    uHistory.resize(1, 0.0);
    
    // Initialize TD_obj with a dummy object
    std::vector<double> dummy_params = {0.9};
    TD_obj = new TD("euler", 0.01, dummy_params);
}

// Destructor
ADRC::~ADRC() {
    if (TD_obj != nullptr) {
        delete TD_obj;
    }
}

// Initialize method
void ADRC::initialize(double Tsettle_val, 
                      double kob_val, 
                      double b0_val,
                      double uMin_val,
                      double uMax_val,
                      double dT_val,
                      std::vector<double> XhatInit,
                      double uInit,
                      double Ke_val,
                      double inputDelay,
                      std::string TD_method,
                      std::vector<double> TD_params,
                      bool useCascaded_val) {
    
    if (Tsettle_val <= 0) throw std::invalid_argument("Tsettle must be positive");
    if (kob_val <= 0) throw std::invalid_argument("kob must be positive");
    if (dT_val <= 0) throw std::invalid_argument("dT must be positive");
    if (Ke_val <= 0) throw std::invalid_argument("Ke must be positive");
    if (inputDelay < 0) throw std::invalid_argument("inputDelay must be non-negative");
    
    // Store parameters
    Tsettle = Tsettle_val;
    kob = kob_val;
    b0 = b0_val / Ke_val;
    Ke = Ke_val;
    dT = dT_val;
    uMin = uMin_val;
    uMax = uMax_val;
    useCascaded = useCascaded_val;
    
    // Initialize state
    if (XhatInit.empty()) {
        xhat.assign(NADRC, 0.0);
    } else {
        if ((int)XhatInit.size() != NADRC) {
            throw std::invalid_argument("XhatInit must have length " + std::to_string(NADRC));
        }
        xhat = XhatInit;
    }
    
    mhat.assign(NADRC, 0.0);
    uPrev = uInit;
    
    // Configure input delay
    inputDelaySteps = std::max(0, (int)std::round(inputDelay / dT));
    uHistory.assign(inputDelaySteps + 1, uInit);
    
    // Compute ESO gains and matrices
    computeESOGains();
    computeControllerGains();
    
    // Setup TD if requested
    if (TD_obj != nullptr) {
        delete TD_obj;
    }
    
    std::transform(TD_method.begin(), TD_method.end(), TD_method.begin(), ::tolower);
    if (TD_method != "none") {
        useTD = true;
        TD_obj = new TD(TD_method, dT, TD_params);
        TD_obj->reset(0.0);
    } else {
        useTD = false;
        std::vector<double> default_params = {0.9};
        TD_obj = new TD("euler", dT, default_params);
    }
    
    isInitialized = true;
}

// Step method
double ADRC::step(double reference, double output, std::vector<double> refDerivatives) {
    if (!isInitialized) {
        throw std::runtime_error("Controller must be initialized before use");
    }
    
    // Update TD if enabled
    double refFiltered;
    std::vector<double> refDerivs;
    
    if (useTD) {
        TD_obj->update(reference);
        refFiltered = TD_obj->y;
        refDerivs.push_back(TD_obj->yd);
        if (n >= 2) {
            refDerivs.push_back(TD_obj->ydd);
        }
    } else {
        refFiltered = reference;
    }
    
    // Parse optional reference derivatives
    if (!refDerivatives.empty()) {
        refDerivs = refDerivatives;
    }
    
    // Update ESO (using delayed control input)
    double u_for_eso = uHistory[0];
    
    // xhat = Ad * xhat + Bd * u_for_eso + Ld * output
    std::vector<double> temp(NADRC, 0.0);
    for (int i = 0; i < NADRC; i++) {
        for (int j = 0; j < NADRC; j++) {
            temp[i] += Ad_data[i * NADRC + j] * xhat[j];
        }
        temp[i] += Bd[i] * u_for_eso + Ld[i] * output;
    }
    xhat = temp;
    
    // Update LESO2 (mhat) if cascaded
    if (useCascaded) {
        std::vector<double> B(NADRC, 0.0);
        B[n - 1] = dT;  // n-1 is the last state before f
        
        std::vector<double> temp_m(NADRC, 0.0);
        for (int i = 0; i < NADRC; i++) {
            for (int j = 0; j < NADRC; j++) {
                temp_m[i] += Ad_data[i * NADRC + j] * mhat[j];
            }
            temp_m[i] += Bd[i] * u_for_eso + Ld[i] * output + B[i] * xhat.back();
        }
        mhat = temp_m;
    } else {
        mhat = xhat;
    }
    
    // Build reference state vector
    std::vector<double> r_states(n, 0.0);
    r_states[0] = refFiltered;
    
    if (!refDerivs.empty()) {
        int m = std::min(n - 1, (int)refDerivs.size());
        for (int i = 0; i < m; i++) {
            r_states[1 + i] = refDerivs[i];
        }
    }
    
    // nth derivative for feedforward
    double ref_nth = 0.0;
    if ((int)refDerivs.size() >= n) {
        ref_nth = refDerivs[n - 1];
    }
    
    // Control law with disturbance rejection
    double u;
    if (useCascaded) {
        // u = (dot(K, r_states - m_states) + ref_nth - mhat[-1] - xhat[-1]) / b0
        double dotK = 0.0;
        for (int i = 0; i < n; i++) {
            dotK += K[i] * (r_states[i] - mhat[i]);
        }
        u = (dotK + ref_nth - mhat.back() - xhat.back()) / b0;
    } else {
        // u = dot([K; 1], RefVec - xhat) / b0
        double dotK = 0.0;
        for (int i = 0; i < n; i++) {
            dotK += K[i] * (r_states[i] - xhat[i]);
        }
        dotK += 1.0 * (ref_nth - xhat.back());
        u = dotK / b0;
    }
    
    // Apply saturation
    u = saturate(u);
    
    // Update control history for delay emulation
    if (uHistory.size() > 1) {
        for (size_t i = 0; i < uHistory.size() - 1; i++) {
            uHistory[i] = uHistory[i + 1];
        }
        uHistory.back() = u;
    } else {
        uHistory[0] = u;
    }
    
    uPrev = u;
    return u;
}

// Reset method
void ADRC::reset(std::vector<double> XhatInit, double uInit) {
    if (XhatInit.empty()) {
        xhat.assign(NADRC, 0.0);
    } else {
        xhat = XhatInit;
    }
    
    mhat.assign(NADRC, 0.0);
    uPrev = uInit;
    uHistory.assign(inputDelaySteps + 1, uInit);
    
    if (useTD && TD_obj != nullptr) {
        TD_obj->reset(0.0);
    }
}

// Set TD method
void ADRC::setTD(std::string method, std::vector<double> params) {
    std::transform(method.begin(), method.end(), method.begin(), ::tolower);
    if (method == "none") {
        useTD = false;
    } else {
        useTD = true;
        if (TD_obj != nullptr) {
            delete TD_obj;
        }
        TD_obj = new TD(method, dT, params);
        TD_obj->reset(0.0);
    }
}

// Set saturation
void ADRC::setSaturation(double uMin_val, double uMax_val) {
    uMin = uMin_val;
    uMax = uMax_val;
}

// Set input delay
void ADRC::setInputDelay(double delaySec) {
    int newDelaySteps = std::max(0, (int)std::round(delaySec / dT));
    
    if (newDelaySteps != inputDelaySteps) {
        if (newDelaySteps == 0) {
            uHistory.assign(1, uPrev);
        } else {
            uHistory.assign(newDelaySteps + 1, uPrev);
        }
        inputDelaySteps = newDelaySteps;
    }
}

// Update tuning
void ADRC::updateTuning(double Tsettle_val, double kob_val) {
    Tsettle = Tsettle_val;
    kob = kob_val;
    
    // Recompute gains
    computeESOGains();
    computeControllerGains();
}

// Get estimated states
std::vector<double> ADRC::getEstimatedStates() {
    return xhat;
}

// Get estimated disturbance
double ADRC::getEstimatedDisturbance() {
    if (useCascaded) {
        return mhat.back() + xhat.back();
    } else {
        return xhat.back();
    }
}

// Get m states
std::vector<double> ADRC::getMStates() {
    return mhat;
}

// Get z states
std::vector<double> ADRC::getZStates() {
    return xhat;
}

// Private: Compute ESO gains
void ADRC::computeESOGains() {
    // Continuous-time ESO matrices
    std::vector<double> A(NADRC * NADRC, 0.0);
    std::vector<double> B(NADRC, 0.0);
    std::vector<double> C(NADRC, 0.0);
    
    for (int i = 0; i < n; i++) {
        A[i * NADRC + (i + 1)] = 1.0;
    }
    B[n - 1] = b0;
    C[0] = 1.0;
    
    // Discretize using matrix exponential
    std::vector<double> Ad_full(NADRC * NADRC);
    expm(A, Ad_full, dT);
    
    std::vector<double> sum_matrix(NADRC * NADRC, 0.0);
    std::vector<double> A_power(NADRC * NADRC, 0.0);
    // Initialize A_power as identity
    for (int i = 0; i < NADRC; i++) {
        A_power[i * NADRC + i] = 1.0;
    }
    
    for (int i = 1; i <= n; i++) {
        // Add (A^(i-1) * dt^i / i!) to sum_matrix
        double coef = std::pow(dT, i) / factorial(i);
        for (int r = 0; r < NADRC; r++) {
            for (int c = 0; c < NADRC; c++) {
                sum_matrix[r * NADRC + c] += A_power[r * NADRC + c] * coef;
            }
        }
        // Update A_power = A_power * A for next iteration
        if (i < n) {
            std::vector<double> temp(NADRC * NADRC, 0.0);
            for (int r = 0; r < NADRC; r++) {
                for (int c = 0; c < NADRC; c++) {
                    for (int k = 0; k < NADRC; k++) {
                        temp[r * NADRC + c] += A_power[r * NADRC + k] * A[k * NADRC + c];
                    }
                }
            }
            A_power = temp;
        }
    }
    
    // Observer pole placement
    std::vector<double> Ld(NADRC, 0.0);
    double Scl, SESO, ZESO;
    
    switch (n) {
        case 1:
            Scl = -4.0 / Tsettle;
            SESO = kob * Scl;
            ZESO = std::exp(SESO * dT);
            Ld[0] = 1.0 - ZESO * ZESO;
            Ld[1] = (1.0 - ZESO) * (1.0 - ZESO) / dT;
            break;
            
        case 2:
            Scl = -6.0 / Tsettle;
            SESO = kob * Scl;
            ZESO = std::exp(SESO * dT);
            Ld[0] = 1.0 - ZESO * ZESO * ZESO;
            Ld[1] = (1.0 + ZESO) * ((1.0 - ZESO) * (1.0 - ZESO)) * 3.0 / (2.0 * dT);
            Ld[2] = ((1.0 - ZESO) * (1.0 - ZESO) * (1.0 - ZESO)) / (dT * dT);
            break;
            
        case 3:
            Scl = -8.0 / Tsettle;
            SESO = kob * Scl;
            ZESO = std::exp(SESO * dT);
            Ld[0] = 1.0 - ZESO * ZESO * ZESO * ZESO;
            Ld[1] = ((1.0 - ZESO) * (1.0 - ZESO)) * (11.0 + ZESO * (14.0 + 11.0 * ZESO)) / (6.0 * dT);
            Ld[2] = ((1.0 - ZESO) * (1.0 - ZESO) * (1.0 - ZESO)) * (1.0 + ZESO) * 2.0 / (dT * dT);
            Ld[3] = ((1.0 - ZESO) * (1.0 - ZESO) * (1.0 - ZESO) * (1.0 - ZESO)) / (dT * dT * dT);
            break;
            
        case 4:
            Scl = -10.0 / Tsettle;
            SESO = kob * Scl;
            ZESO = std::exp(SESO * dT);
            Ld[0] = 1.0 - ZESO * ZESO * ZESO * ZESO * ZESO;
            Ld[1] = ((1.0 - ZESO) * (1.0 - ZESO)) * (1.0 + ZESO) * 
                    (5.0 + ZESO * (2.0 + 5.0 * ZESO)) * 5.0 / (12.0 * dT);
            Ld[2] = ((1.0 - ZESO) * (1.0 - ZESO) * (1.0 - ZESO)) * 
                    (7.0 + ZESO * (10.0 + 7.0 * ZESO)) * 5.0 / (12.0 * dT * dT);
            Ld[3] = ((1.0 - ZESO) * (1.0 - ZESO) * (1.0 - ZESO) * (1.0 - ZESO)) * 
                    (1.0 + ZESO) * 5.0 / (2.0 * dT * dT * dT);
            Ld[4] = ((1.0 - ZESO) * (1.0 - ZESO) * (1.0 - ZESO) * (1.0 - ZESO) * (1.0 - ZESO)) / 
                    (dT * dT * dT * dT);
            break;
    }
    
    // Store ESO matrices
    this->Ld = Ld;
    
    // Ad = Ad_full - Ld * C * Ad_full
    std::vector<double> CAd_full(NADRC * NADRC, 0.0);
    for (int i = 0; i < NADRC; i++) {
        for (int j = 0; j < NADRC; j++) {
            CAd_full[i * NADRC + j] = C[i] * Ad_full[i * NADRC + j];
        }
    }
    
    Ad_data.resize(NADRC * NADRC);
    for (int i = 0; i < NADRC; i++) {
        for (int j = 0; j < NADRC; j++) {
            Ad_data[i * NADRC + j] = Ad_full[i * NADRC + j] - Ld[i] * CAd_full[i * NADRC + j];
        }
    }
    
    // Bd = B * dT
    Bd.resize(NADRC);
    for (int i = 0; i < NADRC; i++) {
        Bd[i] = B[i] * dT;
    }
    
    Cd = C;
}

// Private: Compute controller gains
void ADRC::computeControllerGains() {
    double Scl = -(2.0 * (n + 1)) / Tsettle;
    
    K.resize(n);
    for (int i = 1; i <= n; i++) {
        K[n - i] = (factorial(n) / (factorial(n - i) * factorial(i))) * std::pow(-Scl, i);
    }
}

// Private: Saturation function
double ADRC::saturate(double u) {
    if (!std::isfinite(uMin)) {
        return std::min(u, uMax);
    } else if (!std::isfinite(uMax)) {
        return std::max(u, uMin);
    } else {
        return std::max(std::min(u, uMax), uMin);
    }
}

// Private: Factorial helper
int ADRC::factorial(int x) {
    int result = 1;
    for (int i = 2; i <= x; i++) {
        result *= i;
    }
    return result;
}

// Private: Matrix exponential (simplified using Taylor series)
void ADRC::expm(const std::vector<double>& A, std::vector<double>& result, double dt) {
    int size = NADRC;
    // Scale A by dt
    std::vector<double> A_scaled = A;
    for (size_t i = 0; i < A_scaled.size(); i++) {
        A_scaled[i] *= dt;
    }
    
    // Initialize result as identity matrix
    result.assign(size * size, 0.0);
    for (int i = 0; i < size; i++) {
        result[i * size + i] = 1.0;
    }
    
    // Use truncated Taylor series: expm(A) ≈ I + A + A^2/2! + A^3/3! + ...
    std::vector<double> term = result;  // Start with identity
    for (int k = 1; k <= 10; k++) {    // 10 terms should be sufficient
        std::vector<double> temp(size * size, 0.0);
        for (int r = 0; r < size; r++) {
            for (int c = 0; c < size; c++) {
                for (int k2 = 0; k2 < size; k2++) {
                    temp[r * size + c] += term[r * size + k2] * A_scaled[k2 * size + c];
                }
            }
        }
        term = temp;
        
        double fact = 1.0 / factorial(k);
        for (size_t i = 0; i < temp.size(); i++) {
            result[i] += term[i] * fact;
        }
    }
}