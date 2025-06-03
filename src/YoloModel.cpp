
#include "YoloModel.h"
#include "plog/Log.h"

YoloModel::YoloModel()
{

}

YoloModel::~YoloModel()
{

}

bool YoloModel::init()
{
    LOGD << "YoloModel initialized!"; 
    return true; 
}

bool YoloModel::preprocess(const cv::Mat& inputImage, cv::Mat& outputBlob)
{

}

bool YoloModel::postprocess(const cv::Mat& netOutput, std::shared_ptr<IModelOutput>& output)
{
    
}