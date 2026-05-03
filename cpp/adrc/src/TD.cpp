#include "adrc/TD.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iostream>

// Constructor
TD::TD(std::string method, double dt, std::vector<double> params) 
    : y(0.0), yd(0.0), ydd(0.0) {
    
    if (dt <= 0) {
        throw std::invalid_argument("Sample time dt must be positive");
    }
    
    std::transform(method.begin(), method.end(), method.begin(), ::tolower);
    this->method = method;
    this->dt = dt;
    this->params = params;
    
    // Initialize method-specific state
    _initialize_state(0.0);
}

// Initialize state based on method
void TD::_initialize_state(double initial_value) {
    if (method == "euler") {
        state["prev_y"] = initial_value;
        state["prev_yd"] = 0.0;
        y = initial_value;
        yd = 0.0;
        ydd = 0.0;
    }
    else if (method == "tod") {
        state["v1"] = initial_value;
        state["v2"] = 0.0;
        y = initial_value;
        yd = 0.0;
        ydd = 0.0;
    }
    else if (method == "ld") {
        state["v1"] = initial_value;
        state["v2"] = 0.0;
        y = initial_value;
        yd = 0.0;
        ydd = 0.0;
    }
    else if (method == "red") {
        state["v1"] = initial_value;
        state["v2"] = 0.0;
        y = initial_value;
        yd = 0.0;
        ydd = 0.0;
    }
    else if (method == "intd") {
        state["v1"] = initial_value;
        state["v2"] = 0.0;
        y = initial_value;
        yd = 0.0;
        ydd = 0.0;
    }
    else {
        throw std::invalid_argument("Unknown TD method: " + method);
    }
}

// Update method dispatcher
void TD::update(double ref) {
    if (method == "euler") {
        _update_euler(ref);
    }
    else if (method == "tod") {
        _update_tod(ref);
    }
    else if (method == "ld") {
        _update_ld(ref);
    }
    else if (method == "red") {
        _update_red(ref);
    }
    else if (method == "intd") {
        _update_intd(ref);
    }
}

// Reset method
void TD::reset(double initial_value) {
    _initialize_state(initial_value);
}

// Set parameters
void TD::set_parameters(std::vector<double> params) {
    this->params = params;
}

// Set sample time
void TD::set_sample_time(double dt) {
    if (dt <= 0) {
        throw std::invalid_argument("Sample time dt must be positive");
    }
    this->dt = dt;
}

// Euler-based low-pass filter with derivative estimation
void TD::_update_euler(double ref) {
    /*
     * Euler-based low-pass filter with derivative estimation.
     * Parameter: a (filter coefficient, default 0.9)
     */
    double a = !params.empty() ? params[0] : 0.9;
    
    // Low pass filter
    y = a * state["prev_y"] + (1.0 - a) * ref;
    
    // Euler approximation for first derivative
    yd = (y - state["prev_y"]) / dt;
    
    // Euler approximation for second derivative
    ydd = (yd - state["prev_yd"]) / dt;
    
    // Update state
    state["prev_y"] = y;
    state["prev_yd"] = yd;
}

// Time Optimal Differentiator
void TD::_update_tod(double ref) {
    /*
     * Time Optimal Differentiator.
     * Parameter: r (convergence rate, default 1)
     */
    double r = !params.empty() ? params[0] : 1.0;
    
    // Compute derivatives
    double dot_v1 = state["v2"];
    double dot_v2 = -r * ((state["v1"] - ref + 
                          (state["v2"] * std::abs(state["v2"])) / (2.0 * r)) > 0 ? 1.0 : -1.0);
    
    // Forward Euler integration
    state["v1"] = state["v1"] + dot_v1 * dt;
    state["v2"] = state["v2"] + dot_v2 * dt;
    
    // Outputs
    y = state["v1"];
    yd = state["v2"];
    ydd = 0.0;  // Second derivative not estimated
}

// Linear Differentiator
void TD::_update_ld(double ref) {
    /*
     * Linear Differentiator.
     * Parameter: lambda (bandwidth parameter, default 1)
     */
    double lambda_param = !params.empty() ? params[0] : 1.0;
    
    // Compute derivatives
    double dot_v1 = state["v2"] - (state["v1"] - ref) / lambda_param;
    double dot_v2 = -2.0 * (state["v1"] - ref) / lambda_param;
    
    // Forward Euler integration
    state["v1"] = state["v1"] + dot_v1 * dt;
    state["v2"] = state["v2"] + dot_v2 * dt;
    
    // Outputs
    y = state["v1"];
    yd = state["v2"];
    ydd = 0.0;  // Second derivative not estimated
}

// Robust Exact Differentiator
void TD::_update_red(double ref) {
    /*
     * Robust Exact Differentiator.
     * Parameters: lambda1, lambda2 (both default to 1)
     */
    double lambda1, lambda2;
    if (params.size() < 2) {
        lambda1 = 1.0;
        lambda2 = 1.0;
    } else {
        lambda1 = params[0];
        lambda2 = params[1];
    }
    
    // Compute derivatives
    double err = state["v1"] - ref;
    double dot_v1 = state["v2"] - lambda1 * std::sqrt(std::abs(err)) * (err > 0 ? 1.0 : -1.0);
    double dot_v2 = -lambda2 * (err > 0 ? 1.0 : -1.0);
    
    // Forward Euler integration
    state["v1"] = state["v1"] + dot_v1 * dt;
    state["v2"] = state["v2"] + dot_v2 * dt;
    
    // Outputs
    y = state["v1"];
    yd = state["v2"];
    ydd = 0.0;  // Second derivative not estimated
}

// Improved Nonlinear Tracking Differentiator
void TD::_update_intd(double ref) {
    /*
     * Improved Nonlinear Tracking Differentiator.
     * Parameters: alpha, beta, gamma, r (defaults: 1, 1, 1, 1)
     */
    double alpha, beta, gamma, r;
    if (params.size() < 4) {
        alpha = 1.0;
        beta = 1.0;
        gamma = 1.0;
        r = 1.0;
    } else {
        alpha = params[0];
        beta = params[1];
        gamma = params[2];
        r = params[3];
    }
    
    // Parameter validation
    if (alpha < 0 || alpha > 1) {
        std::cerr << "Warning: alpha should be between 0 and 1 for INTD method, clamping" << std::endl;
        alpha = std::max(0.0, std::min(1.0, alpha));
    }
    if (beta <= 0 || gamma <= 0 || r <= 0) {
        throw std::invalid_argument("beta, gamma, and r must be positive for INTD method");
    }
    
    // Compute derivatives
    double dot_v1 = state["v2"];
    double dot_v2 = -r * r * std::tanh((beta * state["v1"] - (1.0 - alpha) * ref) / gamma) 
                    - r * state["v2"];
    
    // Forward Euler integration
    state["v1"] = state["v1"] + dot_v1 * dt;
    state["v2"] = state["v2"] + dot_v2 * dt;
    
    // Outputs
    y = state["v1"];
    yd = state["v2"];
    ydd = 0.0;  // Second derivative not estimated
}