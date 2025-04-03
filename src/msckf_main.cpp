#include <ros/ros.h>

#include "msckf/msckf_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "msckf_node");
    std::cout << "=== Starting MSCKF Node ===" << std::endl;

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    MSCKF_MONO::MSCKF_Node node(nh, private_nh);



    ros::spin();
    return 0;
}