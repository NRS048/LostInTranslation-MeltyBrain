#ifndef KalmanFilter_h
#define kalman_h

#include "Arduino.h"

class KalmanFilter {
public:
    KalmanFilter(double initial_state, double initial_estimate_error, double process_noise, double measurement_noise);

    // Predict the next state
    void predict();

    // Update the state estimate based on a new measurement
    void update(double measurement);

    // Get the current state estimate
    double getState() const;

private:
    const double A = 1.0;
    const double H = 1.0;
    
    double x_hat_;
    double P_;
    double Q_;
    double R_;
    double K;
};

#endif