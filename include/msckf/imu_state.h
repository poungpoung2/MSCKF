#ifndef IMU_STATE_H
#define IMU_STATE_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <random>

#include "msckf/utils.h"

namespace IMU
{
    typedef long long int state_id;

    struct IMUState
    {
        ros::Time timestamp;
        state_id id;
        static Eigen::Vector3d gravity;
        // Noise parameters
        static double gyro_noise;
        static double acc_noise;
        static double gyro_bias_noise;
        static double acc_bias_noise;
    
        Eigen::Vector3d gyro_bias;
        Eigen::Vector3d acc_bias;


      

        double gyro_noise_sig, accel_noise_sig;
        double gyro_bias_sig, accel_bias_sig;
        double frequency;

        // IMU Quaternion
        Eigen::Vector4d orientation;
        Eigen::Vector4d orientation_null;

        // Global position and velocity
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d velocity_null;

        // Biases in gyro and accelerometer
        Eigen::Vector3d b_g;
        Eigen::Vector3d b_a;

        // Camera IMU extrinsics
        Eigen::Vector4d q_C;
        Eigen::Vector3d p_C;

        Eigen::Matrix3d C_R_I;
        Eigen::Matrix<double, 12, 12> Q_noise;

        IMUState()
        {
            timestamp = ros::Time::now();
            id = 0;
            orientation = Eigen::Vector4d(0, 0, 0, 1); // Fixed initialization
            velocity = Eigen::Vector3d::Zero();
            //position = Eigen::Vector3d::Zero(); 
            position = Eigen::Vector3d(7.710364, 0.407604, -0.750771);
            orientation_null = Eigen::Vector4d(0, 0, 0, 1); // Fixed syntax
            velocity_null = Eigen::Vector3d::Zero();
            gyro_bias= Eigen::Vector3d::Zero();
            acc_bias = Eigen::Vector3d::Zero();

        }
    };

} // namespace IMU

#endif