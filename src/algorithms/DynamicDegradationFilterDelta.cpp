#include "algorithms/DynamicDegradationFilterDelta.hpp"
#include <iostream>
#include <gsl/gsl_randist.h>

void DynamicDegradationFilterDelta::LaplaceApproximationParams::Reset()
{
    FillRatioReference = -1.0;
    PredictionMean = 0.0;
    PredictionStdDev = -1.0;
    NumBlackTilesSeen = 0;
    NumObservations = 0;
}

void DynamicDegradationFilterDelta::TruncationParameters::Init()
{
    // nothing to be done
}

void DynamicDegradationFilterDelta::TruncationParameters::Reset()
{
    StandardizedBounds = {0.0, 0.0};
    TransformedStateEstimate = {0.0, 0.0};
    NormalizationFactor = 0.0;
}

void DynamicDegradationFilterDelta::TruncationParameters::StandardizeBounds(DDPair original_bounds, double mean, double std_dev)
{
    StandardizedBounds = {(original_bounds.first - mean) / std_dev,
                          (original_bounds.second - mean) / std_dev};
}

void DynamicDegradationFilterDelta::TruncationParameters::ComputeNormalizationFactor()
{
    // Calculate the denominator first
    NormalizationFactor = std::erf(StandardizedBounds.second / M_SQRT2) - std::erf(StandardizedBounds.first / M_SQRT2);

    // Prevent an infinite normalization factor (roundoff error); the usage of ZERO_APPROX squared is arbitrary since it represents a very small number
    if (NormalizationFactor < std::numeric_limits<double>::epsilon())
    {
        NormalizationFactor = std::sqrt(M_2_PI) / std::numeric_limits<double>::epsilon();
    }
    else
    {
        NormalizationFactor = std::sqrt(M_2_PI) / NormalizationFactor;
    }
}

void DynamicDegradationFilterDelta::TruncationParameters::ComputeTransformedStateEstimates()
{
    TransformedStateEstimate.first = NormalizationFactor *
                                     (std::exp(-std::pow(StandardizedBounds.first, 2) / 2) - std::exp(-std::pow(StandardizedBounds.second, 2) / 2));
    TransformedStateEstimate.second = NormalizationFactor *
                                          (std::exp(-std::pow(StandardizedBounds.first, 2) / 2) * (StandardizedBounds.first - 2 * TransformedStateEstimate.first) -
                                           std::exp(-std::pow(StandardizedBounds.second, 2) / 2) * (StandardizedBounds.second - 2 * TransformedStateEstimate.first)) +
                                      std::pow(TransformedStateEstimate.first, 2) + 1;
}

/******************************************************************/
/******************************************************************/

DynamicDegradationFilterDelta::DynamicDegradationFilterDelta(const std::shared_ptr<CollectivePerception> &col_per_ptr,
                                                             double lower_bound /* 0.5 + ZERO_APPROX */,
                                                             double upper_bound /* 1.0 - ZERO_APPROX */)
    : ExtendedKalmanFilter(),
      MAP_optimizer_(nlopt::LN_NELDERMEAD, 1),
      bounds_original_({lower_bound, upper_bound}),
      bounds_internal_({lower_bound * SENSOR_ACCURACY_INTERNAL_UNIT_FACTOR_DELTA, upper_bound * SENSOR_ACCURACY_INTERNAL_UNIT_FACTOR_DELTA})
{
    collective_perception_algo_ptr_ = col_per_ptr;
}

void DynamicDegradationFilterDelta::Reset()
{
    // Call base Reset
    ExtendedKalmanFilter::Reset();

    // Clear buffer for informed estimate (if used)
    collective_perception_algo_ptr_->GetParamsPtr()->InformedEstimateHistory.clear();

    // Reset parameters
    truncation_params_.Reset();
    lap_params_.Reset();
}

void DynamicDegradationFilterDelta::Init()
{
    // Initialize the internal unit factor
    internal_unit_factor_ = SENSOR_ACCURACY_INTERNAL_UNIT_FACTOR_DELTA;

    // Specify filter variant type
    if (params_ptr_->FilterSpecificParams["variant"] == "bin")
    {
        variant_ = Variant::BinomialApproximation;
    }
    else if (params_ptr_->FilterSpecificParams["variant"] == "lap")
    {
        variant_ = Variant::LaplaceApproximation;
    }
    else
    {
        throw std::invalid_argument("Unknown variant requested for the Delta filter; only \"bin\" and \"lap\" is allowed.");
    }

    // Initialize common variables
    double init_mean, init_var;

    init_mean = std::stod(params_ptr_->FilterSpecificParams["init_mean"]) * internal_unit_factor_;
    init_var = std::stod(params_ptr_->FilterSpecificParams["init_var"]) * internal_unit_factor_ * internal_unit_factor_;

    initial_guess_ = {init_mean, init_var};

    // Initialize state prediction parameters (common across the variants)
    linearized_state_prediction_a_ = 1.0;
    linearized_state_prediction_b_ = std::stod(params_ptr_->FilterSpecificParams["pred_deg_model_B"].c_str()) * internal_unit_factor_;
    state_prediction_r_ = std::stod(params_ptr_->FilterSpecificParams["pred_deg_var_R"].c_str()) * internal_unit_factor_ * internal_unit_factor_;

    // Populate the nonlinear state prediction function of the EKF (it's actually linear)
    nonlinear_state_prediction_function_ = [this](double prev_mean, const std::vector<double> &input_coefficients)
    {
        return prev_mean +
               this->params_ptr_->FilterActivationPeriodTicks * this->linearized_state_prediction_b_;
    };

    // Initialize measurement update parameters
    linearized_measurement_update_c_ = collective_perception_algo_ptr_->GetParamsPtr()->NumObservations *
                                       (2 * bin_params_.FillRatioReference - 1) / internal_unit_factor_; // t * (2f - 1); needs to be updated every time step
    measurement_update_q_ = collective_perception_algo_ptr_->GetParamsPtr()->NumObservations *
                            (std::pow(2 * params_ptr_->AssumedSensorAcc["b"] / internal_unit_factor_ - 1, 2) * (bin_params_.FillRatioReference - std::pow(bin_params_.FillRatioReference, 2) - 0.25) +
                             0.25); // q = t * ( bf + (1-b)*(1-f)) * (1 - bf - (1-b)*(1-f)) )
                                    //   = t * ( (2*b - 1)^2 * (f - f^2 - 1/4) + 1/4 ); needs to be updated every time step

    // Initialize parameters based on the variants
    switch (variant_)
    {
    case Variant::BinomialApproximation:
    {
        // Populate the nonlinear measurement update function of the EKF (it's actually linear)
        nonlinear_measurement_update_function_ = [this](double pred_mean, const std::vector<double> &input_coefficients)
        {
            // n = t * ( bf + (1-b)*(1-f) )
            //   = t * ( f * (2*b - 1) - b + 1 )
            return this->collective_perception_algo_ptr_->GetParamsPtr()->NumObservations *
                   (this->bin_params_.FillRatioReference * (2 * pred_mean / this->internal_unit_factor_ - 1) - pred_mean / this->internal_unit_factor_ + 1);
        };

        break;
    }
    case Variant::LaplaceApproximation:
    {
        // Set up the MAP estimation function
        auto nlopt_wrapped_MAP_fcn = [](const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) -> double
        {
            LaplaceApproximationParams *dist_params = static_cast<LaplaceApproximationParams *>(my_func_data);

            double success_probability = x[0] / dist_params->InternalUnitFactor * dist_params->FillRatioReference +
                                         (1 - x[0] / dist_params->InternalUnitFactor) * (1 - dist_params->FillRatioReference);

            return gsl_ran_binomial_pdf(dist_params->NumBlackTilesSeen, success_probability, dist_params->NumObservations) *
                   gsl_ran_ugaussian_pdf((x[0] - dist_params->PredictionMean) / dist_params->PredictionStdDev);
        };

        MAP_optimizer_.set_max_objective(nlopt_wrapped_MAP_fcn, &lap_params_);
        MAP_optimizer_.set_xtol_abs(1e-12);

        // Initialize the internal unit factor (only need to be done once)
        lap_params_.InternalUnitFactor = internal_unit_factor_;
        break;
    }
    }

    // Initialize truncation parameters (used for the "post-processing" of the EKF outputs)
    truncation_params_.Init();

    // Populate initial guess
    update_ = initial_guess_;
}

void DynamicDegradationFilterDelta::Predict()
{
    // Run the base predict step
    ExtendedKalmanFilter::Predict(empty_vec_double_);
}

void DynamicDegradationFilterDelta::Update()
{
    switch (variant_)
    {
    case Variant::BinomialApproximation:
    {
        bin_params_.FillRatioReference =
            params_ptr_->UseWeightedAvgInformedEstimates
                ? collective_perception_algo_ptr_->GetParamsPtr()->WeightedAverageInformedEstimate
                : collective_perception_algo_ptr_->GetInformedVals().X;

        // Use np >= 5 && nq >= 5 rule to see if normal approximation holds
        if (collective_perception_algo_ptr_->GetParamsPtr()->NumObservations * (prediction_.first / internal_unit_factor_) >= bin_params_.ThresholdCount &&
            collective_perception_algo_ptr_->GetParamsPtr()->NumObservations * (1 - prediction_.first / internal_unit_factor_) >= bin_params_.ThresholdCount)
        {
            // Update the measurement jacobian and noise models
            linearized_measurement_update_c_ = collective_perception_algo_ptr_->GetParamsPtr()->NumObservations *
                                               (2 * bin_params_.FillRatioReference - 1) / internal_unit_factor_;
            measurement_update_q_ = collective_perception_algo_ptr_->GetParamsPtr()->NumObservations *
                                    (std::pow(2 * prediction_.first / internal_unit_factor_ - 1, 2) * (bin_params_.FillRatioReference - std::pow(bin_params_.FillRatioReference, 2) - 0.25) +
                                     0.25);

            // Run the base update step
            ExtendedKalmanFilter::Update(collective_perception_algo_ptr_->GetParamsPtr()->NumBlackTilesSeen, empty_vec_double_);
        }
        else // approximation doesn't hold up well, do nothing
        {
            update_ = prediction_;
        }
        break;
    }

    case Variant::LaplaceApproximation:
    {
        lap_params_.FillRatioReference = collective_perception_algo_ptr_->GetParamsPtr()->WeightedAverageInformedEstimate;
        lap_params_.PredictionMean = prediction_.first;
        lap_params_.PredictionStdDev = std::sqrt(prediction_.second);
        lap_params_.NumBlackTilesSeen = collective_perception_algo_ptr_->GetParamsPtr()->NumBlackTilesSeen;
        lap_params_.NumObservations = collective_perception_algo_ptr_->GetParamsPtr()->NumObservations;

        // Find the MAP estimate (which becomes the mean of the approximated Gaussian)
        MAP_outcome_.first[0] = prediction_.first;
        MAP_optimization_status_ = MAP_optimizer_.optimize(MAP_outcome_.first, MAP_outcome_.second);

        // Check if optimization is successful
        if (MAP_optimization_status_ < 0)
        {
            std::cout << "MAP Optimization unsuccessful: flag=" << MAP_optimization_status_
                      << " with f(" << MAP_outcome_.first[0] << ") = " << MAP_outcome_.second
                      << std::endl;
        }

        update_.first = MAP_outcome_.first[0]; // populate the updated state estimate

        /* The approximated variance is found using Mathematica:

            FullSimplify[
                1 / -D[
                    Log[
                        (b f + (1 - b) (1 - f))^n (1 - (b f + (1 - b) (1 - f)))^(t - n) Exp[
                            -((b - \[Mu]) / \[Sigma])^2/2
                        ]
                    ], {b, 2}
                ]
            ]

        */
        // Find the variance of the approximateed Gaussian
        double q = std::pow(update_.first + lap_params_.FillRatioReference - 2 * update_.first * lap_params_.FillRatioReference, 2) *
                   std::pow(1 - lap_params_.FillRatioReference + update_.first * (-1 + 2 * lap_params_.FillRatioReference), 2);

        update_.second =
            (q * prediction_.second) /
            (q +
             ((-1 + update_.first * (2 - 4 * lap_params_.FillRatioReference) + 2 * lap_params_.FillRatioReference) * lap_params_.NumBlackTilesSeen + std::pow(-1 + update_.first + lap_params_.FillRatioReference - 2 * update_.first * lap_params_.FillRatioReference, 2) * lap_params_.NumObservations) *
                 std::pow(lap_params_.PredictionStdDev - 2 * lap_params_.FillRatioReference * lap_params_.PredictionStdDev, 2));

        break;
    }
    }
}

void DynamicDegradationFilterDelta::Estimate()
{
    // Execute prediction step
    Predict();

    if (std::isnan(prediction_.first) || std::isnan(prediction_.second))
    {
        throw std::runtime_error("(Variant = " + std::to_string(static_cast<int>(variant_)) + ") NaN values encountered in the predicted values.");
    }

    // Execute update step
    Update();

    if (std::isnan(update_.first) || std::isnan(update_.second))
    {
        throw std::runtime_error("(Variant = " + std::to_string(static_cast<int>(variant_)) + ") NaN values encountered in the updated values.");
    }

    // Find the constrained version of the sensor accuracy; the constrained values are not fed back into the filter
    ComputeConstrainedSensorAccuracy();

    // Update assumed sensor accuracy only if it's out of bounds (should be converted OUT of internal units)
    if (update_.first < bounds_original_.first || update_.first > bounds_original_.second)
    {
        params_ptr_->AssumedSensorAcc["b"] = (std::sqrt(update_.second) * truncation_params_.TransformedStateEstimate.first + update_.first) / internal_unit_factor_;

        // Prevent the assumed sensor accuracy from going outside of bounds (due to rounding errors)
        if (params_ptr_->AssumedSensorAcc["b"] > bounds_original_.second)
        {
            params_ptr_->AssumedSensorAcc["b"] = bounds_original_.second;
        }
        else if (params_ptr_->AssumedSensorAcc["b"] < bounds_original_.first)
        {
            params_ptr_->AssumedSensorAcc["b"] = bounds_original_.first;
        }
    }
    else
    {
        params_ptr_->AssumedSensorAcc["b"] = update_.first;
    }

    // Apply exponential smoothing
    if (prev_assumed_acc_ == -1.0)
    {
        prev_assumed_acc_ = params_ptr_->AssumedSensorAcc["b"];
    }
    else
    {
        params_ptr_->AssumedSensorAcc["b"] = exponential_smoothing_factor_ * params_ptr_->AssumedSensorAcc["b"] + (1 - exponential_smoothing_factor_) * prev_assumed_acc_;
        prev_assumed_acc_ = params_ptr_->AssumedSensorAcc["b"];
    }

    params_ptr_->AssumedSensorAcc["w"] = params_ptr_->AssumedSensorAcc["b"]; // black tile accuracy is equal to white tile accuracy
}

void DynamicDegradationFilterDelta::ComputeConstrainedSensorAccuracy()
{
    // Constrain the assumed sensor accuracy estimate according to
    // "Constrained Kalman filtering via density function truncation for turbofan engine health estimation" by Simon and Simon (2010)

    // Standardize the bounds
    truncation_params_.StandardizeBounds(bounds_internal_, update_.first, std::sqrt(update_.second));

    // Compute normalization factor
    truncation_params_.ComputeNormalizationFactor();

    // Compute transformed state estimates
    truncation_params_.ComputeTransformedStateEstimates();
}