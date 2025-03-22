#include <ros/ros.h>

#include "msclkf/msckf_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "msckf_node");
    std::cout << "=== Starting MSCKF Node ===" << std::endl;

    MSCKF::MSCKF_Node msckf_node;

    ros::spin();
    return 0;
}