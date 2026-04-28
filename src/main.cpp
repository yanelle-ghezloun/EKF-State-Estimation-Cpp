#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <cstdlib>
#include "ekf.h"


// Random number generator
std::default_random_engine gen(42);

double randn(double std) {
    std::normal_distribution<double> dist(0.0, std);
    return dist(gen);
}

int main() {
    // Parameters 
    const double dt       = 0.1;
    const int    steps    = 200;
    const double v        = 1.0;
    const double omega    = 1.0;

    // Init EKF 
    Eigen::Vector3d initial_state(1.0, 0.0, M_PI / 2.0);

    Eigen::Matrix3d initial_cov = Eigen::Matrix3d::Identity() * 0.1;

    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * 0.001;

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.09;

    EKF ekf(initial_state, initial_cov, Q, R);

    Eigen::Vector3d true_state(1.0, 0.0, M_PI / 2.0);
    Eigen::Vector3d odom_state(1.0, 0.0, M_PI / 2.0);

    // Output files 
    std::ofstream f_true("results/true_traj.csv");
    std::ofstream f_ekf("results/ekf_traj.csv");
    std::ofstream f_odom("results/odom_traj.csv");
    std::ofstream f_gps("results/gps_meas.csv");
    std::ofstream f_err("results/errors.csv");

    f_true << "x,y,theta\n";
    f_ekf  << "x,y,theta\n";
    f_odom << "x,y,theta\n";
    f_gps  << "x,y\n";
    f_err  << "error\n";


    f_true << true_state(0) << "," << true_state(1) << "," << true_state(2) << "\n";
    f_ekf  << ekf.getState()(0) << "," << ekf.getState()(1) << "," << ekf.getState()(2) << "\n";
    f_odom << odom_state(0) << "," << odom_state(1) << "," << odom_state(2) << "\n";

    for (int i = 0; i < steps; i++) {

        double v_n     = v     + randn(0.05);
        double omega_n = omega + randn(0.05);

        ekf.predict(v_n, omega_n, dt);

        double theta = true_state(2);
        true_state(0) += v * std::cos(theta) * dt;
        true_state(1) += v * std::sin(theta) * dt;
        true_state(2) += omega * dt;

        double theta_o = odom_state(2);
        odom_state(0) += v_n * std::cos(theta_o) * dt;
        odom_state(1) += v_n * std::sin(theta_o) * dt;
        odom_state(2) += omega_n * dt;

        // GPS update every 3 steps
        if (i % 3 == 0) {
            Eigen::Vector2d z(
                true_state(0) + randn(0.15),
                true_state(1) + randn(0.15)
            );
            ekf.update(z);
            f_gps << z(0) << "," << z(1) << "\n";
        }

        Eigen::Vector3d est = ekf.getState();
        double error = std::sqrt(
            std::pow(est(0) - true_state(0), 2) +
            std::pow(est(1) - true_state(1), 2)
        );

        f_true << true_state(0) << "," << true_state(1) << "," << true_state(2) << "\n";
        f_ekf  << est(0) << "," << est(1) << "," << est(2) << "\n";
        f_odom << odom_state(0) << "," << odom_state(1) << "," << odom_state(2) << "\n";
        f_err  << error << "\n";
    }

    f_true.close();
    f_ekf.close();
    f_odom.close();
    f_gps.close();
    f_err.close();

    std::cout << "Done: results saved in results/" << std::endl;
    std::system("python3 ../plot.py");
    return 0;
}
