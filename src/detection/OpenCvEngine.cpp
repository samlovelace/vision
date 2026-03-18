
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
    LOGD << "OpenCV Inference Engine initialized!"; 
    return true; 
}

bool OpenCvEngine::loadModel(const std::string& aModelPath)
{
    auto toLower = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        return s;
    };

    std::string ext = toLower(aModelPath.substr(aModelPath.find_last_of('.') + 1));

    if (ext == "onnx") 
    {
        mNet = cv::dnn::readNetFromONNX(aModelPath);
    } 
    else if (ext == "pb") 
    {
        // Assumes frozen TensorFlow model
        mNet =  cv::dnn::readNetFromTensorflow(aModelPath);
    } 
    else if (ext == "caffemodel") 
    {
        // Requires accompanying prototxt
        std::string protoPath = aModelPath;
        protoPath.replace(aModelPath.find(".caffemodel"), 11, ".prototxt"); // crude fallback
        mNet =  cv::dnn::readNetFromCaffe(protoPath, aModelPath);
    } 
    else 
    {
        throw std::runtime_error("Unsupported model format: " + ext);
    }

}

cv::Mat OpenCvEngine::doInference(const cv::Mat& anImage)
{
    mNet.setInput(anImage);
    cv::Mat output = mNet.forward();
    return output; 
}