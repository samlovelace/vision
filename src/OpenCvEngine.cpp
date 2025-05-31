
#include "OpenCvEngine.h"
#include "plog/Log.h"

OpenCvEngine::OpenCvEngine()
{

}

OpenCvEngine::~OpenCvEngine()
{

}

bool OpenCvEngine::init()
{
    LOGD << "OPenCV INference Engine init"; 
    return true; 
}

bool OpenCvEngine::loadModel(const std::string& aModelPath)
{

}

void OpenCvEngine::doInference(const cv::Mat& anImage)
{
    
}