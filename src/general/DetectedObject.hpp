#ifndef DETECTEDOBJECT_HPP
#define DETECTEDOBJECT_HPP

#include <string> 
#include <opencv2/opencv.hpp>
#include <chrono> 

struct DetectedObject {
    std::string class_label;        // e.g., "cat"
    std::string instance_id;        // e.g., "cat_001"
    cv::Point3f global_centroid;    // XYZ in world frame
    float confidence;               // optional
    std::chrono::system_clock::time_point last_seen;            // to manage object lifetime
};

#endif