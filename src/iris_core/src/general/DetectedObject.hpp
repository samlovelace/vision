#ifndef DETECTEDOBJECT_HPP
#define DETECTEDOBJECT_HPP

#include <string>
#include <opencv2/opencv.hpp>
#include <chrono>

#include "ObjectPoseFilter.h"

struct DetectedObject
{
    std::string class_label;        // e.g., "cat"
    std::string instance_id;        // e.g., "cat_001"
    cv::Point3f global_centroid;    // XYZ in world frame
    cv::Vec4f   global_orientation; // wxyz quaternion in world frame
    float confidence;               // optional
    std::chrono::system_clock::time_point last_seen;            // to manage object lifetime
    ObjectPoseFilter pose_filter;   // running average + variance of global_centroid/global_orientation

    bool published = false;               // whether this object has ever been published
    cv::Point3f last_published_position;  // pose_filter's mean position at the last publish
};

#endif