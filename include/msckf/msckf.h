#ifndef MSCKF_H
#define MSCKF_H

#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "msckf/imu_state.h"
#include "msckf/camera_state.h"

class TestMSCKF;

namespace MSCKF
{
    class Msckf
    {
        friend class ::TestMSCKF;

    public:
        Msckf(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
            : g(0, 0, -9.80665), w_g(0.0, 0.0, 7.292115e-5),
              F(Eigen::Matrix<double, 15, 15>::Zero()),
              G(Eigen::Matrix<double, 15, 12>::Zero()),
              P_II(Eigen::Matrix<double, 15, 15>::Identity()),
              P_CC(Eigen::MatrixXd(0, 0)),
              P_IC(Eigen::MatrixXd(15, 0)),
              Phi(Eigen::Matrix<double, 15, 15>::Identity())
        {
            imu_state = IMU::IMUState();
            imu_state.loadParams(nh);
            imu_state.timestamp = ros::Time::now();

            cam_state = CAMERA::CameraState();
            cam_state.loadParams(nh);
        }
        ~Msckf() {}

        void propagateState(const sensor_msgs::Imu::ConstPtr &imuMsg);
        void propagateCovariance(double dt);

        // private:
        IMU::IMUState imu_state;
        CAMERA::CameraState cam_state;

        const Eigen::Vector3d w_g; // Angular velocty due to gravity
        const Eigen::Vector3d g;   // Gravity

        Eigen::Matrix<double, 15, 15> F;    // System Jacobian
        Eigen::Matrix<double, 15, 12> G;    // Noise Jacobian
        Eigen::Matrix<double, 15, 15> P_II; // IMU covariance
        Eigen::MatrixXd P_CC<double, cam_state.N>;               // Camera pose covariance
        Eigen::MatrixXd P_IC;               // Cross-correlation

        Eigen::Matrix<double, 15, 15> Phi;  // State transition

        Eigen::Matrix3d crossMatrix(const Eigen::Vector3d &v);
        std::pair<Eigen::Vector3d, Eigen::Vector3d> calculateMeasurement(const IMU::IMUStateVariable &imu_state_var,
                                                                         const Eigen::Matrix3d &C,
                                                                         const Eigen::Vector3d &w,
                                                                         const Eigen::Vector3d &a);
        Eigen::Matrix4d findOmega(const Eigen::Vector3d &w);
        IMU::IMUStateVariable f(const IMU::IMUStateVariable &X, const Eigen::Vector3d &w_I, const Eigen::Vector3d &a_I, bool findFG = false);
        IMU::IMUStateVariable rk5Step(const Eigen::Vector3d &w_I, const Eigen::Vector3d &a_I, double dt);
        
        void computeJacobians(Eigen::Matrix3d C, Eigen::Vector3d w_hat, Eigen::Vector3d a_hat);
        void Msckf::StateAugmentation();

    };
}

#endif