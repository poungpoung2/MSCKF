#include "msckf/feature_tracker.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "msckf/feature.h"

namespace TRACKER {
// Track features from previous frame to current frame using KLT

void TRACKER::initialize(const sensor_msgs::Image::ConstPtr& img_msg) {
    prev_frame = getImage(img_msg);
    addNewFeatures(prevFrame);
}

cv::Mat TRACKER::getImage(const sensor_msgs::Image::ConstPtr& img_msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(current_img, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    return image;
}

void TRACKER::trackFeatures(const sensor_msgs::Image::ConstPtr& current_img) {
    cur_frame = get(image);

    std::vector<cv::Point2f> prev_points;
    std::vector<int> feature_ids;

    for (const auto& [id, point] : sliding_window) {
        feature_ids.push_back(id);
        prev_points.push_back(point);
    }

    std::vector<cv::Point2f> next_points;
    std::vector<uchar> status;
    std::vector<float> err;

    bool triangulate;

    cv::calcOpticalFlowPyrLK(prev_frame, cur_frame, prev_points, next_points,
                             status, err, win_size, max_level, termcrit);

    for (size_t i = 0; i < status.size(); i++) {
        id = feature_ids[i];

        if (status[i]) {
            bool isMax = sliding_window[id].add_observation(next_points[i]);
            prev_points_map[id] = next_points[i];
            if (isMax) {
                triangulate_feature.add(id);
            }
        } else {
            triangulate_feature.add(id);
            prev_points_map.erase(id);
        }
    }
}

// Add new features to maintain a minimum feature count
void TRACKER::addNewFeatures(const cv::Mat& image) {
    int needed_features = max_feature_count - sliding_window.size();

    if (needed_features <= 0) return;
    const int min_distance = 10;

    mask = image.clone();
    const int min_distance = 10;

    for (const auto& [id, point] : prev_points_map) {
        cv::Circle(mask, point, min_distance, 0, -1);
    }

    cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create(20, true);
    std::vector<cv::KeyPoint> keypoints;
    fast->detect(image, keypoints, mask);

    std::sort(keypoints.begin(), keypoints.end(),
              [](const cv::Keypoint& a, const cv::Keypoint& b) {
                  a.response > b.response;
              });

    int added = 0;
    for (const auto& kp : keypoints) {
        if (added >= needed_features) break;
        if (mask.at<uchar>(p.y, p.x)) {
            ::Point2f p = kp.pt;

            feature = Feature(feature_count, p, ros::Time::now());
            sliding_window[feature_count] = feature;
            prev_points_map[feature_count] = p;
            added++;
            feature_count++;

            cv::circle(mask, min_distance, 0, -1);
        }
    }
}

}  // namespace TRACKER
