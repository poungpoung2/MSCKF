#ifndef IMU_STATE_H
#define IMU_STATE_H

#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <random>

namespace IMU
{
    // Create class to save the IMU states
    struct IMUStateVariable
    {
        // IMU Quaternion
        Eigen::Quaterniond q;

        // Global position and velocity
        Eigen::Vector3d v;
        Eigen::Vector3d p;

        // Biases in gyro and accelerometer
        Eigen::Vector3d b_g;
        Eigen::Vector3d b_a;
    };

    // Override the addition opperation for the state
    inline IMUStateVariable operator+(const IMUStateVariable &A, const IMUStateVariable &B)
    {
        IMUStateVariable result;

        // Combine quaternion rotation
        result.q = (A.q * B.q).normalized();
        result.v = A.v + B.v;
        result.p = A.p + B.p;
        result.b_a = A.b_a + B.b_a;
        result.b_g = A.b_g + B.b_g;

        return result;
    }

    // Overrided the multiplication operation
    inline IMUStateVariable operator*(double dt, const IMUStateVariable &dX)
    {
        IMUStateVariable result;

        // Extract angular displacement
        Eigen::Vector3d omega = 2.0 * dX.q.vec();
        // Compuate total rotation angle
        double angle = omega.norm() * dt;
        if (angle > 1e-6)
        {
            // For large angle extract rotation axis
            Eigen::Vector3d axis = omega.normalized();
            // Create the delta rotation quaternion
            result.q = Eigen::AngleAxisd(angle, axis);
        }
        else
        {
            // Use small angle appriomation
            result.q.w() = 1.0;
            result.q.vec() = 0.5 * omega * dt;
        }

        result.v = dt * dX.v;
        result.p = dt * dX.p;
        result.b_g = dt * dX.b_g;
        result.b_a = dt * dX.b_a;

        return result;
    }

    inline IMUStateVariable operator-(const IMUStateVariable &A, const IMUStateVariable &B)
    {
        IMUStateVariable result;
        result.q = A.q * B.q.conjugate();
        result.v = A.v - B.v;
        result.p = A.p - B.p;
        result.b_g = A.b_g - B.b_g;
        result.b_a = A.b_a - B.b_a;
        return result;
    }

    struct IMUState
    {
        ros::Time timestamp;

        Eigen::Vector3d n_g, n_a;
        Eigen::Matrix<double, 12, 12> Q;

        // IMU Quaternion
        IMUStateVariable state;

        double gyro_noise_sig, accel_noise_sig, gryo_bias_sig, accel_bias_sig, frequency;

        void loadParams(const ros::NodeHandle &nh)
        {
            ros::NodeHandle imu_nh(nh, "imu");
            double gyro_noise, accel_noise, gyro_bias, accel_bias;

            imu_nh.param("noise/gyro_density", gyro_noise, 0.001);
            imu_nh.param("noise/accel_density", accel_noise, 0.01);
            imu_nh.param("noise/gyro_bias_instability", gyro_bias, 0.001);
            imu_nh.param("noise/accel_bias_instability", accel_bias, 0.02);

            imu_nh.param("frequency", frequency, 200.0);

            gyro_noise_sig = sqrt((gyro_noise * gyro_noise) * frequency / 2.0);
            accel_noise_sig = sqrt((accel_noise * accel_noise) * frequency / 2.0);

            gryo_bias_sig = sqrt((gyro_bias * gyro_bias) / frequency);
            accel_bias_sig = sqrt((accel_bias * accel_bias) / frequency);

            Q.block<3, 3>(0, 0) = (gyro_noise_sig * gyro_noise_sig) * Eigen::Matrix3d::Identity();
            Q.block<3, 3>(3, 3) = (gryo_bias_sig * gryo_bias_sig) * Eigen::Matrix3d::Identity();
            Q.block<3, 3>(6, 6) = (accel_noise_sig * accel_noise_sig) * Eigen::Matrix3d::Identity();
            Q.block<3, 3>(9, 9) = (accel_bias_sig * accel_bias_sig) * Eigen::Matrix3d::Identity();
        }

        void sampleNoise()
        {
            thread_local static std::mt19937 gen(std::random_device{}());

            std::normal_distribution<double> gyro_noise_dist(0.0, gyro_noise_sig);
            std::normal_distribution<double> accel_noise_dist(0.0, accel_noise_sig);

            for (int i = 0; i < 3; i++)
            {
                n_g[i] = gyro_noise_dist(gen);
                n_a[i] = accel_noise_dist(gen);
            }
        }

        IMUState()
        {
            state.q = Eigen::Quaterniond::Identity();
            state.v = Eigen::Vector3d::Zero();
            state.p = Eigen::Vector3d::Zero();
            state.b_g = Eigen::Vector3d::Zero();
            state.b_a = Eigen::Vector3d::Zero();
            n_g = Eigen::Vector3d::Zero();
            n_a = Eigen::Vector3d::Zero();
        }
    };
}

#endif