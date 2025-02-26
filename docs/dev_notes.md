# Dev notes
- The dynamic topology static deg (DTSD) experiment is run by using the `kheperaiv_diffusion_motion_controller` and the `sensor_degradation_filter_loop_functions` libraries.
- In simulation, the assumed accuracy of a robot (if it's supposed to have a different one from its actual accuracy) is set in the `<loop_functions>` node. This is because:
    - we can't control --- at least not easily --- which robots are flawed and which aren't if we assign the values using the controller,
    - if we want to use the same controller for real robots, we can technically cross-compile specific robots to have flawed assumed accuracies, but using loop functions is way more streamlined
    - the actual accuracy of the robot is set in the `<controllers>` node because it applies to all the robots.

- The `StaticDegradationAlpha` class refers to the static degradation filter without adaptive activation. The results from using this filter is available but not included in the paper.
- The `StaticDegradationBravo` class refers to the ASDF filter; it's simply `StaticDegradationAlpha` with the activation mechanism. The paper "_Adaptive Self-Calibration for Minimalistic Collective Perception using Imperfect Robot Swarms_" discusses the proposal of this filter.
- The `DynamicDegradationCharlie` class refers to the dynamic degradation filter that does posterior approximation using pure vanilla VI. What this means is that the distribution family is fixed and we optimize the ELBO for the distribution parameters.
    - The degradation model is assumed to follow a Wiener process with drift $\eta$ and variance $\sigma^2$:

        $x(t) = x(0) + \eta t + \sigma W(t)$ where $W(t)$ is the standard Wiener process (a.k.a. standard Brownian motion).

    - Thus the prediction model used in this filter is as follows.

        ```cpp
        x_pred[k+1] = x_pos[k] + B * (k+1 - k); \\ predicted mean with assumed drift of B
        var_pred[k+1] = var_pos[k] + R; \\ predicted variance with assumed diffusion variance of R
        ```
    - The observations `n` can be kept in a queue (since it's non-stationary). What this means is that the past `N` observations are kept in this queue. If an observation queue is used then the fill ratio reference used by the binomial likelihood (_i.e._, the informed estimate) is computed as a weighted average of past `N` informed estimates.
    - The predicted estimates are checked and modified so that they stay within the bounds before being fed into the update step, after which bounds checking is applied again.

- The `tests/test_elbo_class.cpp` script is provided to test the GSL integration functionality and NLopt optimization functionality.

- The `DynamicDegradationDelta` class is a method that uses the ExtendedKalmanFilter to estimate the sensor accuracy and then apply the truncation based on the linear inequality constraints _only_ if the estimates violate the constraints. This highlights the main distinction from Charlie: the estimates are not modified within the EKF but after estimates are done. The modified estimates are also _not_ fed back into the filter here.
    - The terms "window size" and "queue size" _do not_ mean the same thing. The window size is used to find the moving average value of the degradation rate (so that we get dynamic observation queue sizes; this applies to the Charlie filter as well). The queue size strictly applies to the storage of observations (and past informed estimates, if requested).




- Each `*DegradationJsonData` object contains data from multiple `num_flawed_robots` experiments (all trials of that experiment is included). The top-level key is `num_flawed_robots`. _E.g.,_ if there are two cases of `num_flawed_robots` within the same directory of JSON files, then all those files will be stored into one `*DegradationJsonData` object.

- The `vicon_sdk` folder and its contents were copied from the [NESTLab/Vicon repository](https://github.com/NESTLab/Vicon/tree/38ca8d7b52a7a727e8d37c2fb49c1b2058a8ead7/vicon_sdk/Linux64).

### Analyzing dynamic degradation data
- Procedure for analyzing the dynamic degradation data:
    1. Extract using `extract_dynamic_degradation_data.py`. This will generate a bunch of HDF file pairs, one containing the informed estimate values, the other containing sensor accuracy values.
    2. Compute the RMSD values using `compute_rmsd_df.py`. This will take the HDF files from the previous step to compute and concatenate into a single HDF file with the RMSD values.
    3. Extract the RMSD HDF files into different CSV files with a unique parameter set using `extract_isolated_rmsd_data.py`. Typically the set is ["true_drift_coeff", "lowest_degraded_acc_lvl", "fsp_pred_deg_model_B", "correct_sensor_acc_b", "flawed_sensor_acc_b", "tfr"] where a single CSV file is only for a specific combination of the set.

- This is done to compartmentalize the different parts of the data processing procedure, which should improve debugging (if things go wrong). It also provides a bit of flexibility in accessing data; the first step contains *all* the raw data that can be used differently downstream. The second step contains RMSD data for multiple parameter sets, while the third step generates a specific parameter set's RMSD data ready for data visualization (the CSV makes it easy for debugging also).