#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include "msckf/msckf.h"
#include "msckf/imu_state.h"
#include "msckf/camera_state.h"
#include <stdio.h>
#include <utility>
#include <unsupported/Eigen/MatrixFunctions>

namespace MSCKF
{
    // Create a skew-symmetric matrix from a vector
    Eigen::Matrix3d Msckf::crossMatrix(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d cross;
        cross << 0.0, -v.z(), v.y(),
            v.z(), 0.0, -v.x(),
            -v.y(), v.x(), 0.0;
        return cross;
    }

    // Compute measured angular velocity and linear acceleration
    std::pair<Eigen::Vector3d, Eigen::Vector3d> Msckf::calculateMeasurement(const IMU::IMUStateVariable &imu_state_var, const Eigen::Matrix3d &C, const Eigen::Vector3d &w_I, const Eigen::Vector3d &a_I)
    {
        Eigen::Vector3d w_m;
        Eigen::Vector3d a_m;
        Eigen::Matrix3d crossW = crossMatrix(w_g);

        // ADDED: Log raw inputs
        printf("calculateMeasurement - Raw w: [%.4f, %.4f, %.4f] \n", w_I.x(), w_I.y(), w_I.z());
        printf("calculateMeasurement - Raw a: [%.4f, %.4f, %.4f] \n", a_I.x(), a_I.y(), a_I.z());

        w_m = w_I + C * w_g + imu_state_var.b_g + imu_state.n_g;
        a_m = a_I + C * (2 * crossW * imu_state_var.v + crossW * crossW * imu_state_var.p) + imu_state_var.b_a + imu_state.n_a;

        // ADDED: Log computed measurements
        printf("Computed w_m: [%.4f, %.4f, %.4f] \n", w_m.x(), w_m.y(), w_m.z());
        printf("Computed a_m: [%.4f, %.4f, %.4f] \n", a_m.x(), a_m.y(), a_m.z());

        return {w_m, a_m};
    }

    // Construct Omega matrix for quaternion derivative
    Eigen::Matrix4d Msckf::findOmega(const Eigen::Vector3d &w)
    {
        Eigen::Matrix4d Omega;
        Omega.block<3, 3>(0, 0) = -crossMatrix(w);
        Omega.block<3, 1>(0, 3) = w;
        Omega.block<1, 3>(3, 0) = -w.transpose();
        Omega(3, 3) = 0.0;
        return Omega;
    }

    void Msckf::computeJacobians(Eigen::Matrix3d C, Eigen::Vector3d w_hat, Eigen::Vector3d a_hat)
    {
        F.block<3, 3>(0, 0) = -crossMatrix(w_hat);
        F.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
        F.block<3, 3>(6, 0) = -C.transpose() * crossMatrix(a_hat);
        F.block<3, 3>(6, 6) = -2 * crossMatrix(w_g);
        F.block<3, 3>(6, 9) = -C.transpose();
        F.block<3, 3>(6, 12) = -crossMatrix(w_g) * crossMatrix(w_g);
        F.block<3, 3>(12, 6) = Eigen::Matrix3d::Identity();

        G.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
        G.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
        G.block<3, 3>(6, 6) = -C.transpose();
        G.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();

        // ADDED: Jacobian matrix logging
        printf("F matrix (top-left 3x3): \n");
        for (int i = 0; i < 3; ++i)
        {
            printf("[%.4f, %.4f, %.4f] \n",
                   F(i, 0), F(i, 1), F(i, 2));
        }
        printf("G matrix diagonal: [%.4f, %.4f, %.4f] \n",
               G(0, 0), G(1, 1), G(2, 2));
    }

    // System dynamics function
    IMU::IMUStateVariable Msckf::f(const IMU::IMUStateVariable &X, const Eigen::Vector3d &w_I, const Eigen::Vector3d &a_I, bool findFG)
    {
        printf("Initial position: [%.4f, %.4f, %.4f]\n", X.p.x(), X.p.y(), X.p.z());
        Eigen::Vector3d w_m, a_m;
        Eigen::Vector3d w_hat, a_hat;
        Eigen::Matrix4d Omega;

        Eigen::Matrix3d crossW = crossMatrix(w_g);
        Eigen::Matrix3d C = X.q.toRotationMatrix();

        std::tie(w_m, a_m) = calculateMeasurement(X, C, w_I, a_I);

        w_hat = w_m - X.b_g - C * w_g;
        a_hat = a_m - X.b_a;

        if (findFG)
            computeJacobians(C, w_hat, a_hat);

        Omega = findOmega(w_hat);

        IMU::IMUStateVariable dX;
        dX.q = Eigen::Quaterniond(0.5 * Omega * X.q.coeffs()).normalized();
        dX.p = X.v;
        dX.v = C.transpose() * a_hat - 2 * crossW * X.v - crossW * crossW * X.p + g;
        dX.b_g = Eigen::Vector3d::Zero();
        dX.b_a = Eigen::Vector3d::Zero();
        dX.q.normalize();

        // ADDED: State derivative logging
        printf("State derivatives:");
        printf("dq: [%.6f, %.6f, %.6f, %.6f] \n",
               dX.q.w(), dX.q.x(), dX.q.y(), dX.q.z());
        printf("dv: [%.6f, %.6f, %.6f] \n",
               dX.v.x(), dX.v.y(), dX.v.z());

        return dX;
    }

    // RK5 integration step
    IMU::IMUStateVariable Msckf::rk5Step(const Eigen::Vector3d &w_I, const Eigen::Vector3d &a_I, double dt)
    {
        imu_state.sampleNoise();
        IMU::IMUStateVariable X = imu_state.state;

        // ADDED: RK step logging
        printf("Starting RK5 step with dt=%.6f \n", dt);

        IMU::IMUStateVariable k1 = f(X, w_I, a_I, true);
        IMU::IMUStateVariable k2 = f(X + (dt / 5.0) * k1, w_I, a_I);
        IMU::IMUStateVariable k3 = f(X + dt * ((3.0 / 40.0) * k1 + (9.0 / 40.0) * k2), w_I, a_I);
        IMU::IMUStateVariable k4 = f(X + dt * ((44.0 / 45.0) * k1 - (56.0 / 15.0) * k2 + (32.0 / 9.0) * k3), w_I, a_I);
        IMU::IMUStateVariable k5 = f(X + dt * ((19372.0 / 6561.0) * k1 - (25360.0 / 2187.0) * k2 + (64448.0 / 6561.0) * k3 - (212.0 / 729.0) * k4), w_I, a_I);
        IMU::IMUStateVariable k6 = f(X + dt * ((9017.0 / 3168.0) * k1 - (355.0 / 33.0) * k2 + (46732.0 / 5247.0) * k3 + (49.0 / 176.0) * k4 - (5103.0 / 18656.0) * k5), w_I, a_I);

        IMU::IMUStateVariable X_next = X + dt * ((35.0 / 384.0) * k1 + (500.0 / 1113.0) * k3 + (125.0 / 192.0) * k4 - (2187.0 / 6784.0) * k5 + (11.0 / 84.0) * k6);

        // ADDED: State update logging
        printf("New state position: [%.4f, %.4f, %.4f] \n",
               X_next.p.x(), X_next.p.y(), X_next.p.z());

        return X_next;
    }

    void Msckf::propagateCovariance(double dt)
    {
        Eigen::Matrix<double, 15, 15> A = F;
        Phi = (A * dt).exp();

        Eigen::Matrix<double, 15, 15> deltaP_II = F * P_II + P_II * F.transpose() + G * imu_state.Q * G.transpose();
        Eigen::Matrix<double, 15, 15> deltaPhi = F * Phi;

        P_II = P_II + deltaP_II * dt;
        Phi = Phi + deltaPhi * dt;
        P_IC = Phi * P_IC;

        // ADDED: Covariance logging
        ROS_INFO("Covariance propagation complete \n");
        printf("P_II trace: %.6f \n", P_II.trace());
        printf("Phi matrix norm: %.6f \n", Phi.norm());
    }

    void Msckf::propagateState(const sensor_msgs::Imu::ConstPtr &imuMsg)
    {
        double dt = imuMsg->header.stamp.toSec() - imu_state.timestamp.toSec();

        // ADDED: IMU input logging
        ROS_INFO("New IMU message received");
        printf("IMU dt: %.6f seconds \n", dt);
        printf("Raw angular: [%.6f, %.6f, %.6f] \n",
               imuMsg->angular_velocity.x,
               imuMsg->angular_velocity.y,
               imuMsg->angular_velocity.z);
        printf("Raw linear: [%.6f, %.6f, %.6f] \n",
               imuMsg->linear_acceleration.x,
               imuMsg->linear_acceleration.y,
               imuMsg->linear_acceleration.z);

        Eigen::Vector3d w_I(
            imuMsg->angular_velocity.x,
            imuMsg->angular_velocity.y,
            imuMsg->angular_velocity.z);
        Eigen::Vector3d a_I(
            imuMsg->linear_acceleration.x,
            imuMsg->linear_acceleration.y,
            imuMsg->linear_acceleration.z);

        IMU::IMUStateVariable X_next = rk5Step(w_I, a_I, dt);

        imu_state.state = X_next;
        imu_state.timestamp = imuMsg->header.stamp;

        // ADDED: Final state logging
        ROS_INFO("State propagation complete \n");
        printf("New position: [%.4f, %.4f, %.4f] \n",
               X_next.p.x(), X_next.p.y(), X_next.p.z());
        printf("New velocity: [%.4f, %.4f, %.4f] \n",
               X_next.v.x(), X_next.v.y(), X_next.v.z());
    }

    void Msckf::StateAugmentation()
    {
        cam_state.q_G = imu_state.state.q * cam_state.q_I;
        cam_state.q_G.normalize();
        
        Eigen::Matrix3d I_C_G = imu_state.state.q.toRotationMatrix().transpose();

        cam_state.p_G = imu_state.state.p + I_C_G * cam_state.p_I;

        int N = cam_state.N;
        int rows_J = 6; 
        int cols_J = 15 + 6 * N; 

        int rows_aug = 15 + 6 * (N + 1); // 21 + 6N
        int cols_aug = 15 + 6 * N;

        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(rows_J, cols_J);
        Eigen::Matrix3d cross_term = I_C_G * crossMatrix(cam_state.p_I);
        J.block<3, 3>(0, 0) = cam_state.q_I.toRotationMatrix();
        J.block<3, 3>(3, 0) = cross_term;
        J.block<3, 3>(3, 15 + 6*N - 3) = Eigen::Matrix3d::Identity(); 


        Eigen::MatrixXd U = Eigen::MatrixXd::Identity(rows_aug, cols_aug);
        U.bottomRows(6) = J;

        Eigen::MatrixX(rows_aug, rows_aug) P;
        P.setZero();

        P.block(0, 0, 15, 15) = P_II;
        P.block(0, 15, 15, 6*N) = P_IC;
        P.block(15, 0, 6*N, 15) = P_IC.transpose();
        P.block(15, 15, 6*N, 6*N) = P_CC;

        P = U * P * U.transpose();

        P_II = P.block(0, 0, 15, 15);               // IMU covariance
        P_IC = P.block(0, 15, 15, 6 * N);           // Cross-covariance
        P_CC = P.block(15, 15, 6 * N, 6 * N);       // Camera covariance

        cam_state.N += 1;
    }
}