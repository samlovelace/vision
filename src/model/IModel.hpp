#ifndef IMODEL_H
#define IMODEL_H

#include <opencv2/core.hpp>
#include "IModelOutput.hpp"

class IModel 
{ 
public:
    virtual ~IModel() = default; 

    virtual bool init() = 0;

    // Accepts a raw image and returns a blob for inference
    virtual bool preprocess(const cv::Mat& inputImage, cv::Mat& outputBlob) = 0;

    // Accepts the output from the network and processes the detections
    virtual bool postprocess(const cv::Mat& netOutput, std::shared_ptr<IModelOutput>& output) = 0;

    std::pair<int, int> getInputSize() {return mInputSize;}

protected:   
    std::pair<int, int> mInputSize; 
};

#endif // IMODEL_H
