#include "OnnxEngine.h"
#include <opencv2/imgproc.hpp>
#include <iostream>

OnnxEngine::OnnxEngine()
    : mEnv(ORT_LOGGING_LEVEL_WARNING, "Inference"), mSessionOptions()
{
    mSessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
}

bool OnnxEngine::init()
{
    return true; // nothing special here for now
}

bool OnnxEngine::loadModel(const std::string& aModelPath)
{
    try {
        mSession = std::make_unique<Ort::Session>(mEnv, aModelPath.c_str(), mSessionOptions);

        size_t numInputs = mSession->GetInputCount();
        size_t numOutputs = mSession->GetOutputCount();

        mInputNamesStr.clear();
        mInputNames.clear();
        mOutputNamesStr.clear();
        mOutputNames.clear();

        // Get input names
        for (size_t i = 0; i < numInputs; ++i) {
            Ort::AllocatedStringPtr name = mSession->GetInputNameAllocated(i, mAllocator);
            mInputNamesStr.emplace_back(name.get()); // copy to string vector
            mInputNames.push_back(mInputNamesStr.back().c_str());
        }

        // Get output names
        for (size_t i = 0; i < numOutputs; ++i) {
            Ort::AllocatedStringPtr name = mSession->GetOutputNameAllocated(i, mAllocator);
            mOutputNamesStr.emplace_back(name.get());
            mOutputNames.push_back(mOutputNamesStr.back().c_str());
        }

        // Get input shape
        auto inputInfo = mSession->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo();
        mInputShape = inputInfo.GetShape();

        return true;
    }
    catch (const Ort::Exception& e) {
        std::cerr << "ONNX Runtime loadModel failed: " << e.what() << std::endl;
        return false;
    }
}


cv::Mat OnnxEngine::doInference(const cv::Mat& anImage)
{
    try {
        // Convert input to float32 and resize to model input size
        cv::Mat resized, inputBlob;
        int height = static_cast<int>(mInputShape[2]);
        int width = static_cast<int>(mInputShape[3]);

        cv::resize(anImage, resized, cv::Size(width, height));
        resized.convertTo(inputBlob, CV_32F, 1.0 / 255);

        // Change layout HWC -> CHW
        std::vector<cv::Mat> channels(3);
        cv::split(inputBlob, channels);
        std::vector<float> inputTensorValues;
        for (int c = 0; c < 3; ++c)
            inputTensorValues.insert(inputTensorValues.end(), (float*)channels[c].data, (float*)channels[c].data + width * height);

        size_t inputTensorSize = inputTensorValues.size();
        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
            memoryInfo, inputTensorValues.data(), inputTensorSize, mInputShape.data(), mInputShape.size());

        // Run inference
        auto outputTensors = mSession->Run(Ort::RunOptions{nullptr},
                                           mInputNames.data(), &inputTensor, 1,
                                           mOutputNames.data(), 1);

        // Extract output tensor
        float* outputData = outputTensors[0].GetTensorMutableData<float>();
        auto outputShape = outputTensors[0].GetTensorTypeAndShapeInfo().GetShape();
        int outH = outputShape.size() >= 3 ? outputShape[2] : 1;
        int outW = outputShape.size() >= 4 ? outputShape[3] : 1;
        int outC = outputShape.size() >= 2 ? outputShape[1] : 1;

        // Convert to cv::Mat (assuming 3D output)
        cv::Mat output(outH, outW, CV_32FC(outC), outputData);
        return output.clone(); // clone to copy from internal buffer

    } catch (const Ort::Exception& e) {
        std::cerr << "ONNX Runtime inference failed: " << e.what() << std::endl;
        return {};
    }
}
