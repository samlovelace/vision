#ifndef IMODEL_H
#define IMODEL_H

#include <opencv2/core.hpp>

class IModel 
{ 
public:
    virtual ~IModel() = default; 

    virtual bool init() = 0;

    // Accepts a raw image and returns a blob for inference
    virtual bool preprocess(const cv::Mat& inputImage, cv::Mat& outputBlob) = 0;

    // Accepts the output from the network and processes the detections
    virtual bool postprocess(const cv::Mat& netOutput, std::vector<cv::Rect>& boxes) = 0;
};

#endif // IMODEL_H
