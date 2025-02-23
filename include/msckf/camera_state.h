#ifndef CAMERA_STATE_H
#define CAMERA_STATE_H

#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <random>

namespace CAMERA
{
    struct CameraState
    {
        // Camera rotation and position in global frame
        Eigen::Quaterniond q_G;
        Eigen::Vector3d p_G;

        //
        Eigen::Quaterniond q_I;
        Eigen::Vector3d p_I;

        int N_MAX;
        int N;


        void loadParams(const ros::NodeHandle &nh)
        {
            ros::NodeHandle cam_nh(nh, "camera/extrinsics");

            std::vector<double> rot;
            cam_nh.param("rotation", rot, [ 0.707, 0.0, 0.707, 0.0 ]);
            q_I = Eigen::Quaterniond(rot[0], rot[1], rot[2], rot[3]).normalized();

            std::vector<double> trans;
            cam_nh.param("translation", trans, [ 0.1, 0.0, 0.05 ]);
            p_I = Eigen::Vector3d(trans[0], trans[1], trans[2]);

            cam_nh.param("N_MAX", N_MAX, 3);
        }

        CameraState(){
            q_G = Eigen::Quaterniond::Identity();
            p_G = Eigen::Vector3d::Zero();
        }
    };
}

#endif