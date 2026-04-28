#include "ekf.h"
#include <cmath>

EKF::EKF(const Eigen::Vector3d& initial_state,
         const Eigen::Matrix3d& initial_cov,
         const Eigen::Matrix3d& process_noise,
         const Eigen::Matrix2d& meas_noise)
    : x(initial_state), P(initial_cov), Q(process_noise), R(meas_noise) {}

void EKF::predict(double v, double omega, double dt) {
    double theta = x(2);

    // Non linear motion model
    x(0) += v * std::cos(theta) * dt;
    x(1) += v * std::sin(theta) * dt;
    x(2) += omega * dt;

    // Jacobian of motion model (linearization)
    Eigen::Matrix3d F;
    F << 1, 0, -v * std::sin(theta) * dt,
         0, 1,  v * std::cos(theta) * dt,
         0, 0,  1;

    // Propagate covariance
    P = F * P * F.transpose() + Q;
}

void EKF::update(const Eigen::Vector2d& z) {
    // Observation matrix
    Eigen::Matrix<double, 2, 3> H;
    H << 1, 0, 0,
         0, 1, 0;

    Eigen::Vector2d y = z - H * x;
    Eigen::Matrix2d S = H * P * H.transpose() + R;
    Eigen::Matrix<double, 3, 2> K = P * H.transpose() * S.inverse();
    x = x + K * y;
    P = (Eigen::Matrix3d::Identity() - K * H) * P;
}
