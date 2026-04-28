#pragma once
#include <Eigen/Dense>

/**
 * Extended Kalman Filter for 2D robot pose estimation
 * State vector: [x, y, theta]
 */
class EKF {
public:
    // State and covariance
    Eigen::Vector3d x;  // [x, y, theta]
    Eigen::Matrix3d P;  // covariance matrix

    // Noise matrices
    Eigen::Matrix3d Q;  // process noise
    Eigen::Matrix2d R;  // measurement noise

    /**
     * Constructor
     * @param initial_state  [x, y, theta]
     * @param initial_cov    initial covariance matrix
     * @param process_noise  process noise matrix
     * @param meas_noise     measurement noise matrix
     */
    EKF(const Eigen::Vector3d& initial_state,
        const Eigen::Matrix3d& initial_cov,
        const Eigen::Matrix3d& process_noise,
        const Eigen::Matrix2d& meas_noise);

    /**
     * Prediction step
     * @param v      linear velocity (m/s)
     * @param omega  angular velocity (rad/s)
     * @param dt     timestep (s)
     */
    void predict(double v, double omega, double dt);

    /**
     * Update step
     * @param z  GPS measurement [x, y]
     */
    void update(const Eigen::Vector2d& z);

    Eigen::Vector3d getState() const { return x; }
    Eigen::Matrix3d getCovariance() const { return P; }
};
