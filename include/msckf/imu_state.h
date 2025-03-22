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

namespace IMU {
typedef long long int state_id;

struct IMUState {
    ros::Time timestamp;
    state_id id;

    Eigen::Vector3d n_g, n_a;

    // Noise parameters
    double gyro_noise_sig, accel_noise_sig, gryo_bias_sig, accel_bias_sig, frequency;
    double gyro_bias, acc_bias;

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
    Eigen::Matrix<double, 12, 12> Q;

    int N_MAX;

    void loadParams(const ros::NodeHandle& nh) {
        // Load camera extrinsics
        ros::NodeHandle cam_nh(nh, "camera/extrinsics");

        std::vector<double> rot;
        cam_nh.param("rotation", rot, {0.707, 0.0, 0.707, 0.0});
        q_C = Eigen::Vector4d(rot[0], rot[1], rot[2], rot[3]).normalized();

        std::vector<double> trans;
        cam_nh.param("translation", trans, {0.1, 0.0, 0.05});
        p_C = Eigen::Vector3d(trans[0], trans[1], trans[2]);

        C_R_I = quaternionToRotation(q_C);

        cam_nh.param("N_MAX", N_MAX, 3);

        // Load IMU parameters
        ros::NodeHandle imu_nh(nh, "imu");

        imu_nh.param("noise/gyro_density", gyro_noise, 0.001);
        imu_nh.param("noise/accel_density", accel_noise, 0.01);
        imu_nh.param("noise/gyro_bias_instability", gyro_bias, 0.001);
        imu_nh.param("noise/accel_bias_instability", acc_bias, 0.02);

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

    void sampleNoise() {
        thread_local static std::mt19937 gen(std::random_device{}());

        std::normal_distribution<double> gyro_noise_dist(0.0, gyro_noise_sig);
        std::normal_distribution<double> accel_noise_dist(0.0, accel_noise_sig);

        for (int i = 0; i < 3; i++) {
            n_g[i] = gyro_noise_dist(gen);
            n_a[i] = accel_noise_dist(gen);
        }
    }

    IMUState() {
        timestamp = 0;
        id = 0;
        orientation = Eigen::Vector4d(0, 0, 0, 1);  // Fixed initialization
        velocity = Eigen::Vector3d::Zero();
        position = Eigen::Vector3d::Zero();  // Fixed spelling
        b_g = Eigen::Vector3d::Zero();
        b_a = Eigen::Vector3d::Zero();
        n_g = Eigen::Vector3d::Zero();
        n_a = Eigen::Vector3d::Zero();
        orientation_null = Eigen::Vector4d(0, 0, 0, 1);  // Fixed syntax
        velocity_null = Eigen::Vector3d::Zero();
    }
};

}  // namespace IMU

#endif