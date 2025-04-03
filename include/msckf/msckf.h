#ifndef MSCKF_H
#define MSCKF_H

#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "msckf/camera_state.h"
#include "msckf/imu_state.h"
#include "msckf/utils.h"

class TestMSCKF;

namespace MSCKF_MONO
{
    typedef long int state_id;
    class MSCKF
    {
    public:
        MSCKF(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : isInitialized(false),
                                                                       state_cov(Eigen::MatrixXd::Zero(21, 21)),
                                                                       noise_cov(Eigen::Matrix<double, 12, 12>::Zero())
        {
            loadParams(nh);
        }

        ~MSCKF() {}
        static Eigen::Vector3d gravity;

        // private:
        IMU::IMUState imu_state;
        CAMERA::CAMState cam_state;

        bool isInitialized;

        std::map<state_id, CAMERA::CAMState> cam_states;
        std::vector<sensor_msgs::Imu> imu_msg_buffer;

        Eigen::Matrix<double, 21, 21> F; // System Jacobian
        Eigen::Matrix<double, 21, 12> G; // Noise Jacobian

        Eigen::MatrixXd state_cov; // IMU covariance
        Eigen::Matrix<double, 12, 12> noise_cov;

        void loadParams(const ros::NodeHandle &nh);
        void initializeGravityAndBias();
        void rk5Step(const double dt, const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc);
        void computeJacobians(Eigen::Matrix3d &I_R_G, Eigen::Vector3d &gyro, Eigen::Vector3d &acc);
        void propagateIMU(double dt, const Eigen::Vector3d &gyro_m, const Eigen::Vector3d &acc_m);
        void applyConstraints(const double dt, const Eigen::Matrix3d &I_R_G, Eigen::Matrix<double, 21, 21> &Phi);
        void propagateCovariance(const double dt, Eigen::Matrix<double, 21, 21> &Phi);
        void propagateState(const sensor_msgs::Imu::ConstPtr &imuMsg);
        void stateAugmentation();
    };
} // namespace MSCKF_MONO

#endif