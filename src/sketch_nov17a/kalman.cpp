#include "kalman.h"

KalmanFilter::KalmanFilter(double initial_state, double initial_estimate_error, double process_noise, double measurement_noise) {
    x_hat_ = initial_state;
    P_ = initial_estimate_error;
    Q_ = process_noise;
    R_ = measurement_noise;
}

void KalmanFilter::predict() {
    x_hat_ = A * x_hat_;
    P_ = A * P_ * A + Q_;
}

void KalmanFilter::update(double measurement) {
    K = P_ / (P_ + R_);
    x_hat_ = x_hat_ + K * (measurement - H * x_hat_);
    P_ = (1 - K * H) * P_;
}

double KalmanFilter::getState() const {
    return x_hat_;
}
