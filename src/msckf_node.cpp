#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include "msckf/msckf.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msckf");
    std::cout << "MSCKF" << std::endl;

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    MSCKF::Msckf msckf_class(nh, pnh);

    ros::spin();
    return 0;
}
