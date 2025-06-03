#ifndef IMODELOUTPUT_HPP
#define IMODELOUTPUT_HPP

#include <vector> 
#include <opencv2/opencv.hpp>

struct IModelOutput {
    virtual ~IModelOutput() = default;
};

struct DetectionOutput : public IModelOutput {
    std::vector<cv::Rect> boxes;
};

struct DepthOutput : public IModelOutput {
    cv::Mat depthMap;
};

#endif