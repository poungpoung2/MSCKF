#include "msckf/feature_tracker.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>


namespace FeatureTracker{

    void Msckf::propagateState(const sensor_msgs::Imu::ConstPtr &imuMsg)


    void FEATURE::detectFeature(const sesnor_msgs::Img:::ConstPtr &imgMsg){

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
        cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(20, true);
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(cv_ptr, keypoints); 



    }

}



