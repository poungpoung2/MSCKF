#ifndef CAMERA_STATE_H
#define CAMERA_STATE_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <random>

#include "msckf/utils.h"

namespace CAMERA {
struct CAMState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // unique cam state id
    state_id id;

    // time when state is recorded
    double time;

    // Orientation of camera, vector from world frame to camera frame
    Eigen::Vector4d orientation;

    // Position of camera in world frame
    Eigen::Vector3d position;

    // Observability constraints
    Eigen::Vector4d orientation_null;
    Eigen::Vector3d position_null;

    CAMState() : id(0), time(0), orientation(Eigen::Vector4d(0, 0, 0, 1)), position(Eigen::Vector3d::Zero()), orientation_null(Eigen::Vector4d(0, 0, 0, 1)), position_null(Eigen::Vector3d::Zero()) {}
};
}  // namespace CAMERA

#endif