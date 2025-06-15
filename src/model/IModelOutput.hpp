#ifndef IMODELOUTPUT_HPP
#define IMODELOUTPUT_HPP

#include <vector> 
#include <opencv2/opencv.hpp>

struct Detection2D
{
    std::string class_name;
    float confidence;
    
    // Either a bounding box OR a mask is valid
    cv::Rect bounding_box;     // (x, y, width, height)
    cv::Mat segmentation_mask; // CV_8UC1, same size as image, 0 or 255
}; 

struct IModelOutput {
    virtual ~IModelOutput() = default;
};

struct DetectionOutput : public IModelOutput 
{
    std::map<std::string, std::vector<Detection2D>> mDetections; 
};

struct DepthOutput : public IModelOutput {
    cv::Mat depthMap;
};

#endif