#ifndef FEATURE_TRACKER_H
#define FEATURE_TRACKER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <unordered_map>

#include "msckf/feature.h"

namespace TRACKER {
class FeatureTracker {
   public:
    std::unordered_map<int, Feature> sliding_window;
    std::unordered_map<int, cv::Point2f> prev_points_map;

    std::vector<int> triangulate_feature;

    int MAX_LEN;
    int feature_count = 0;
    cv::Mat prev_frame, current_frame;

    void TRACKER::initialize(const sensor_msgs::Image::ConstPtr& current_img);

    // Track features from previous frame to current frame using KLT
    void trackFeatures(const sensor_msgs::Image::ConstPtr& current_img);

    // Add new features to maintain a minimum feature count
    void addNewFeatures(const sensor_msgs::Image::ConstPtr& img);

    // Update the sliding window with new camera pose
    void updateSlidingWindow(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);
};
}  // namespace TRACKER

#endif
