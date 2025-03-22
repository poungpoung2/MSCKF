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

namespace MSCKF_MONO {
class MSCKF {
   public:
    void initialize() {
        g(0, 0, -9.80665);
        w_g(0.0, 0.0, 7.292115e-5);
        F(Eigen::Matrix<double, 15, 15>::Zero());
        G(Eigen::Matrix<double, 15, 12>::Zero());
        P_II(Eigen::Matrix<double, 15, 15>::Identity());
        P_CC(Eigen::MatrixXd(0, 0));
        P_IC(Eigen::MatrixXd(15, 0));
        Phi(Eigen::Matrix<double, 15, 15>::Identity());

        
    }

    MSCKF(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) {
        initialize();

        imu_state = IMU::IMUState();
        imu_state.loadParams(nh);
        imu_state.timestamp = ros::Time::now();

        cam_state = CAMERA::CameraState();
        cam_state.loadParams(nh);
    }

    ~Msckf() {}

    std::pair<Eigen::Vector3d, Eigen::Vector3d> prcoessMsg(const sensor_msgs::Imu::ConstPtr &imuMsg);
    void propagateState(const sensor_msgs::Imu::ConstPtr &imuMsg);
    void propagateCovariance(double dt);

    // private:
    IMU::IMUState imu_state;
    CAMERA::CameraState cam_state;

    std::map<state_id, CAMERA::CAM_State> cam_states;

    const Eigen::Vector3d w_g;  // Angular velocty due to gravity
    const Eigen::Vector3d g;    // Gravity

    Eigen::Matrix<double, 15, 15> F;  // System Jacobian
    Eigen::Matrix<double, 15, 12> G;  // Noise Jacobian

    Eigen::MatrixXd state_cov;  // IMU covariance
    Eigen::Matrix<double, 12, 12> noise_cov;

    void rk5Step(const Eigen::Vector3d &w_I, const Eigen::Vector3d &a_I, double dt);
    void computeJacobians(Eigen::Matrix3d &I_R_G, Eigen::Vector3d &gyro, Eigen::Vector3d &acc);
    void propagateIMU(double dt, const Eigen::Vector3d &gyro_m, const Eigen::Vector3d &acc_m);
    void applyConstraints(const double dt, const Eigen::Matrix3d &I_R_G, const Eigen::Matrix<double, 21, 21> &Phi);
    void propagateCovariance(const double dt, const Eigen::Matrix<double, 21, 21> &Phi);
    void propagateState(const sensor_msgs::Imu::ConstPtr &imuMsg);
    void stateAugmentation();
};
}  // namespace MSCKF_MONO

#endif