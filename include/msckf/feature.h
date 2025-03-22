#ifndef FEATURE_H
#define FEATURE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

enum TrackState {
    NEW,
    TRACKED,
    LOST
};

struct Observation {
    cv::Point2f point;
    ros::Time time;

    Observation(const cv::Point2f& initial_obs, const ros::Time& timestamp) {
        point = initial_obs;
        time = timestamp;
    }
};

struct Feature {
    int id;
    int track_count = 0;
    std::vector<Observation> observations;
    TrackState tracking_state;
    bool is_triangulated;

    Feature(int id, cv::Point2f& initial_obs, ros::Time& timestamp) {
        this->id = id;
        track_count = 1;
        observations.push_back(Observation(initial_obs, timestamp));
        tracking_state = TrackState::NEW;
        is_triangulated = false;
        global_pos = Eigen::Vector3d::Zero();  // or NaN
    }

    void add_observation(cv::Point2f obs) {
        track_count += 1;
        observations.push_back(Observation(obs, ros::Time::now()));
        if (track_count > 30) {
            return true;
        } else {
            return false;
        }
    }
};
#endif