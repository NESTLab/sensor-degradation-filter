#ifndef EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_HPP

#include <cmath>
#include <deque>
#include <functional>

#include "SensorDegradationFilter.hpp"

using DDPair = std::pair<double, double>;

class ExtendedKalmanFilter : public SensorDegradationFilter
{
public:
    ExtendedKalmanFilter();

    virtual void Reset() override;

    virtual void Init() = 0; // force initialization so that the nonlinear functions can be populated

protected:
    virtual DDPair Predict(const std::vector<double> &state_prediction_coefficients);

    virtual DDPair Update(double measurement, const std::vector<double> &measurement_update_coefficients);

    virtual void Estimate(double measurement, const std::vector<double> &state_prediction_coefficients, const std::vector<double> &measurement_update_coefficients);

    DDPair initial_guess_ = {0.0, 0.0}; // initial guess to be used for the first prediction step; first: state estimate, second: state variance

    DDPair prediction_ = {0.0, 0.0}; // first: state estimate, second: state variance

    DDPair update_ = {0.0, 0.0}; // first: state estimate, second: state variance

    std::function<double(double, const std::vector<double> &)> nonlinear_state_prediction_function_;

    std::function<double(double, const std::vector<double> &)> nonlinear_measurement_update_function_;

    double exponential_smoothing_factor_ = 1.0; // ignore this for the time being

    double kalman_gain_ = 0.0; // Kalman gain

    double linearized_state_prediction_a_ = 0.0; // linearized coefficient for the state_prediction model (default value: zero)

    double linearized_state_prediction_b_ = 0.0; // linearized coefficient for the state prediction input (aka the drift coefficient) (default: no drift)

    double state_prediction_r_ = -1.0; // our model of what the diffusion coefficient is (default: invalid value; requires setting)

    double linearized_measurement_update_c_ = 0.0; // linearized coefficient for the measurement model (default: not observing; requires setting)

    double measurement_update_q_ = -1.0; // our model of what the measurement noise model is (default: invalid value; requires setting)
};

#endif