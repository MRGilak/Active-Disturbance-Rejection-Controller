#ifndef ADRC_HPP
#define ADRC_HPP

#include <vector>
#include <string>
#include <limits>

class TD;

class ADRC {
private:
    // System configuration
    int n;                      
    double dT;                  
    
    // Gains and matrices
    std::vector<double> Ld;     
    std::vector<double> K;      
    std::vector<double> Ad_data;
    std::vector<double> Beso;
    std::vector<double> Bd;     
    std::vector<double> Cd;     
    int NADRC;                  
    
    // ESO configuration
    double b0;                  
    double omega_c;
    std::vector<double> omega_o;
    int numObservers;
    std::vector<int> estimateLayers;
    
    // State variables
    std::vector<double> observerStates;
    double uPrev;               
    
    // Cascaded ADRC
    bool useCascaded;           
    
    // Saturation
    double uMin;                
    double uMax;                
    
    // Input delay handling
    int inputDelaySteps;        
    std::vector<double> uHistory;
    
    // Tracking differentiator (optional)
    bool useTD;                 
    TD* TD_obj;                 
    
    // Initialization flag
    bool isInitialized;         
    bool explicitObserverGains;

    // Private methods
    void computeESOGains();
    void computeControllerGains();
    double saturate(double u);
    int factorial(int x);
    void expm(const std::vector<double>& A, std::vector<double>& result, double dt);
    std::vector<double> matrixInverse(const std::vector<double>& A);

public:
    // Constructor
    ADRC(int systemOrder);
    
    // Destructor
    ~ADRC();
    
    // Initialize method
    void initialize(double Tsettle_val = 1.0, 
                   double kob_val = 10.0, 
                   double b0_val = 1.0,
                   double uMin_val = -std::numeric_limits<double>::infinity(),
                   double uMax_val = std::numeric_limits<double>::infinity(),
                   double dT_val = 0.01,
                   std::vector<double> XhatInit = {},
                   double uInit = 0.0,
                   double Ke_val = 1.0,
                   double inputDelay = 0.0,
                   std::string TD_method = "none",
                   std::vector<double> TD_params = {},
                   bool useCascaded_val = false);

    void initialize(double omega_c_val, std::vector<double> omega_o_val,
                    double b0_val = 1.0,
                    double uMin_val = -std::numeric_limits<double>::infinity(),
                    double uMax_val = std::numeric_limits<double>::infinity(),
                    double dT_val = 0.01,
                    std::vector<double> XhatInit = {},
                    double uInit = 0.0, double inputDelay = 0.0,
                    std::string TD_method = "none",
                    std::vector<double> TD_params = {},
                    bool useCascaded_val = false,
                    int numObservers_val = 1,
                    std::vector<int> estimateLayers_val = {},
                    std::vector<double> observerGains = {},
                    std::vector<double> controllerGains = {});
    
    // Step method
    double step(double reference, double output, std::vector<double> refDerivatives = {});
    
    // Reset method
    void reset(std::vector<double> XhatInit = {}, double uInit = 0.0);
    
    // Set TD method
    void setTD(std::string method, std::vector<double> params = {});
    
    // Set saturation
    void setSaturation(double uMin_val, double uMax_val);
    
    // Set input delay
    void setInputDelay(double delaySec);
    
    // Update tuning
    void updateTuning(double Tsettle_val, double kob_val);
    
    // Get estimated states
    std::vector<double> getEstimatedStates();
    
    // Get estimated disturbance
    double getEstimatedDisturbance();
    
    std::vector<double> getObserverStates(int layer = 0);
};

#endif // ADRC_HPP