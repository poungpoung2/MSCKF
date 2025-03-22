#ifndef MSCKF_NODE_H
#define MSCKF_NODE_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "msckf/msckf.h"

namespace MSCKF_MONO {

class MSCKF_Node {
   public:
    MSCKF_Node(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    MSCKF_Node() : MSCKF_Node(ros::NodeHandle(), ros::NodeHandle("~")) {}
    ~MSCKF_Node() {}

   private:
    void imu_callback(const sensor_msgs::ImuConstPtr& msg);

    void publish_odom();

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber imu_sub;
    ros::Publisher odom_pub;

    MSCKF filter;
};

}  // namespace MSCKF_MONO

#endif