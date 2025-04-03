#include "msckf/msckf_node.h"

namespace MSCKF_MONO {

MSCKF_Node::MSCKF_Node(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh), filter(nh, pnh) {
    std::cout << "Create IMU Subscriber" << std::endl;
    imu_sub = nh_.subscribe<sensor_msgs::Imu>("dvs/imu", 100, &MSCKF_Node::imu_callback, this);

    std::cout << "Creating Odometry Publisher" << std::endl;
    odom_pub = nh_.advertise<nav_msgs::Odometry>("odom_msckf", 100);

    std::cout << "Initializing Filter" << std::endl;
   // filter.init();
}

void MSCKF_Node::imu_callback(const sensor_msgs::ImuConstPtr& msg) {
    filter.propagateState(msg);
    publish_odom();
}

void MSCKF_Node::publish_odom() {
    nav_msgs::Odometry imu_odom;
    imu_odom.header.frame_id = "world";
    imu_odom.pose.pose.position.x = filter.imu_state.position(0);
    imu_odom.pose.pose.position.y = filter.imu_state.position(1);
    imu_odom.pose.pose.position.z = filter.imu_state.position(2);

    Eigen::Quaterniond q(filter.imu_state.orientation(3), filter.imu_state.orientation(0), filter.imu_state.orientation(1), filter.imu_state.orientation(2));
    Eigen::Quaterniond initial_rotation(0.278892348808751, -0.26918187748657, -0.661557623013424, 0.641951467045268);

    imu_odom.pose.pose.orientation.x = (initial_rotation * q).x();
    imu_odom.pose.pose.orientation.y = (initial_rotation * q).y();
    imu_odom.pose.pose.orientation.z = (initial_rotation * q).z();
    imu_odom.pose.pose.orientation.w = (initial_rotation * q).w();

    odom_pub.publish(imu_odom);
}

} // namespace MSCKF_MONO