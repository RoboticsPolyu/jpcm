#include "gps_imu.h"

void GIO::Process()
{
    /// Replace these with real data
    vector<ImuMeasurement> imu_measurements;
    vector<GpsMeasurement> gps_measurements;

    size_t first_gps_pose = 1;
    size_t gps_skip = 10;  // Skip this many GPS measurements each time
    double g = 9.8;
    auto w_coriolis = Vector3::Zero();  // zero vector

    Vector6 BodyP =
        (Vector6() << gio_params_.body_ptx, gio_params_.body_pty,
                      gio_params_.body_ptz, gio_params_.body_prx,
                      gio_params_.body_pry, gio_params_.body_prz).finished();
    auto body_T_imu = Pose3::Expmap(BodyP);
    if (!body_T_imu.equals(Pose3(), 1e-5)) {
        printf(
            "Currently only support IMUinBody is identity, idx_gps.e. IMU and body frame "
            "are the same");
        exit(-1);
    }

    // Configure noise models
    auto noise_model_gps = noiseModel::Diagonal::Precisions(
        (Vector6() << Vector3::Constant(0), Vector3::Constant(gio_params_.gps_sigma))
            .finished());

    // Set initial conditions for the estimated trajectory
    // initial pose is the reference frame (navigation frame)

    auto current_pose_global = Pose3(Rot3(), gps_measurements[first_gps_pose].position);
    // the vehicle is stationary at the beginning at position 0,0,0
    Vector3 current_velocity_global = Vector3::Zero();
    auto current_bias = imuBias::ConstantBias();  // init with zero bias

    auto sigma_init_x = noiseModel::Diagonal::Precisions(
        (Vector6() << Vector3::Constant(0), Vector3::Constant(gio_params_.gps_sigma)).finished());
    auto sigma_init_v = noiseModel::Diagonal::Sigmas(Vector3::Constant(1000.0));
    auto sigma_init_b = noiseModel::Diagonal::Sigmas(
        (Vector6() << Vector3::Constant(0.100), Vector3::Constant(5.00e-05))
            .finished());

    // Set IMU preintegration parameters
    Matrix33 measured_acc_cov =
        I_3x3 * pow(gio_params_.accelerometer_sigma, 2);
    Matrix33 measured_omega_cov =
        I_3x3 * pow(gio_params_.gyroscope_sigma, 2);
    // error committed in integrating position from velocities
    Matrix33 integration_error_cov =
        I_3x3 * pow(gio_params_.integration_sigma, 2);

    auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(g);
    imu_params->accelerometerCovariance =
        measured_acc_cov;  // acc white noise in continuous
    imu_params->integrationCovariance =
        integration_error_cov;  // integration uncertainty continuous
    imu_params->gyroscopeCovariance =
        measured_omega_cov;  // gyro white noise in continuous
    imu_params->omegaCoriolis = w_coriolis;

    std::shared_ptr<PreintegratedImuMeasurements> current_summarized_measurement =
        nullptr;

    // Set ISAM2 parameters and create ISAM2 solver object
    ISAM2Params isam_params;
    isam_params.factorization = ISAM2Params::CHOLESKY;
    isam_params.relinearizeSkip = 10;

    ISAM2 isam(isam_params);

    // Create the factor graph and values object that will store new factors and
    // values to add to the incremental graph
    NonlinearFactorGraph new_factors;
    Values new_values;  // values storing the initial estimates of new nodes in
                        // the factor graph

    size_t jdx_imu = 0;
    int idx_gps = 0;

    while (true) // waiting gps message
    {
        // At each non=IMU measurement we initialize a new node in the graph
        auto current_pose_key = X(idx_gps);
        auto current_vel_key = V(idx_gps);
        auto current_bias_key = B(idx_gps);
        double t = gps_measurements[idx_gps].time;
        size_t included_imu_measurement_count = 0;

        if (idx_gps == first_gps_pose) 
        {
            // Create initial estimate and prior on initial pose, velocity, and biases
            new_values.insert(current_pose_key, current_pose_global);
            new_values.insert(current_vel_key, current_velocity_global);
            new_values.insert(current_bias_key, current_bias);
            new_factors.emplace_shared<PriorFactor<Pose3>>(
                current_pose_key, current_pose_global, sigma_init_x);
            new_factors.emplace_shared<PriorFactor<Vector3>>(
                current_vel_key, current_velocity_global, sigma_init_v);
            new_factors.emplace_shared<PriorFactor<imuBias::ConstantBias>>(
                current_bias_key, current_bias, sigma_init_b);
        } 
        else 
        {
            double t_previous = gps_measurements[idx_gps - 1].time;

            // Summarize IMU data between the previous GPS measurement and now
            current_summarized_measurement =
                std::make_shared<PreintegratedImuMeasurements>(imu_params,
                                                                current_bias);

            while (jdx_imu < imu_measurements.size() && imu_measurements[jdx_imu].time <= t) 
            {
                if (imu_measurements[jdx_imu].time >= t_previous) 
                {
                    current_summarized_measurement->integrateMeasurement(
                        imu_measurements[jdx_imu].accelerometer, imu_measurements[jdx_imu].gyroscope,
                        imu_measurements[jdx_imu].dt);
                    included_imu_measurement_count++;
                }
                jdx_imu++;
            }

            // Create IMU factor
            auto previous_pose_key = X(idx_gps - 1);
            auto previous_vel_key = V(idx_gps - 1);
            auto previous_bias_key = B(idx_gps - 1);

            new_factors.emplace_shared<ImuFactor>(
                previous_pose_key, previous_vel_key, current_pose_key,
                current_vel_key, previous_bias_key, *current_summarized_measurement);

            // Bias evolution as given in the IMU metadata
            auto sigma_between_b = noiseModel::Diagonal::Sigmas(
                (Vector6() << Vector3::Constant(
                    sqrt(included_imu_measurement_count) *
                    gio_params_.accelerometer_bias_sigma),
                Vector3::Constant(sqrt(included_imu_measurement_count) *
                                    gio_params_.gyroscope_bias_sigma))
                    .finished());
            new_factors.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(
                previous_bias_key, current_bias_key, imuBias::ConstantBias(),
                sigma_between_b);

            // Create GPS factor
            auto gps_pose =
                Pose3(current_pose_global.rotation(), gps_measurements[idx_gps].position);
            if ((idx_gps % gps_skip) == 0) 
            {
                new_factors.emplace_shared<PriorFactor<Pose3>>(
                    current_pose_key, gps_pose, noise_model_gps);
                new_values.insert(current_pose_key, gps_pose);

                printf("############ POSE INCLUDED AT TIME %.6lf ############\n",
                    t);
                cout << gps_pose.translation();
                printf("\n\n");
            } 
            else 
            {
                new_values.insert(current_pose_key, current_pose_global);
            }

            // Add initial values for velocity and bias based on the previous
            // estimates
            new_values.insert(current_vel_key, current_velocity_global);
            new_values.insert(current_bias_key, current_bias);

            // Update solver
            // =======================================================================
            // We accumulate 2*GPSskip GPS measurements before updating the solver at
            // first so that the heading becomes observable.
            if (idx_gps > (first_gps_pose + 2 * gps_skip)) {
                printf("############ NEW FACTORS AT TIME %.6lf ############\n",
                    t);
                new_factors.print();

                isam.update(new_factors, new_values);

                // Reset the newFactors and newValues list
                new_factors.resize(0);
                new_values.clear();

                // Extract the result/current estimates
                Values result = isam.calculateEstimate();

                current_pose_global = result.at<Pose3>(current_pose_key);
                current_velocity_global = result.at<Vector3>(current_vel_key);
                current_bias = result.at<imuBias::ConstantBias>(current_bias_key);

                printf("\n############ POSE AT TIME %lf ############\n", t);
                current_pose_global.print();
                printf("\n\n");
            }
        }
    }
}