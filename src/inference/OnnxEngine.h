#ifndef ONNXENGINE_H
#define ONNXENGINE_H

#include "IInferenceEngine.hpp"
#include <onnxruntime_cxx_api.h>
#include <memory>

class OnnxEngine : public IInferenceEngine
{
public:
    OnnxEngine();
    ~OnnxEngine() = default;

    bool init() override;
    bool loadModel(const std::string& aModelPath) override;
    cv::Mat doInference(const cv::Mat& anImage) override;

private:
    Ort::Env mEnv;
    Ort::SessionOptions mSessionOptions;
    std::unique_ptr<Ort::Session> mSession;
    Ort::AllocatorWithDefaultOptions mAllocator;

    std::vector<int64_t> mInputShape;

    std::vector<std::string> mInputNamesStr;
    std::vector<std::string> mOutputNamesStr;
    std::vector<const char*> mInputNames;
    std::vector<const char*> mOutputNames;

};

#endif // OnnxEngine_H
