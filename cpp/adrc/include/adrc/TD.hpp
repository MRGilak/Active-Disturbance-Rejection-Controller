#ifndef TD_HPP
#define TD_HPP

#include <vector>
#include <string>
#include <map>

class TD {
public:
    // Output properties
    double y;      // Filtered output
    double yd;     // First derivative
    double ydd;    // Second derivative
    
    // Constructor
    TD(std::string method, double dt, std::vector<double> params = {});
    
    // Update method
    void update(double ref);
    
    // Reset method
    void reset(double initial_value = 0.0);
    
    // Set parameters
    void set_parameters(std::vector<double> params);
    
    // Set sample time
    void set_sample_time(double dt);

private:
    std::string method;
    double dt;
    std::vector<double> params;
    std::map<std::string, double> state;
    
    // Initialization
    void _initialize_state(double initial_value);
    
    // Update methods for each differentiator type
    void _update_euler(double ref);
    void _update_tod(double ref);
    void _update_ld(double ref);
    void _update_red(double ref);
    void _update_intd(double ref);
};

#endif // TD_HPP