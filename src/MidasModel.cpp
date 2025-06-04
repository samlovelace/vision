#include "MidasModel.h"
#include <opencv2/dnn.hpp>
#include <iostream>

MidasModel::MidasModel()
{
}

MidasModel::~MidasModel()
{
}

bool MidasModel::init()
{
    // You could load ONNX model here if needed
    return true;
}

bool MidasModel::preprocess(const cv::Mat& aFrame, cv::Mat& anOutput)
{
    // Step 1: Convert BGR (OpenCV default) to RGB
    cv::Mat rgb;
    cv::cvtColor(aFrame, rgb, cv::COLOR_BGR2RGB);

    // Step 2: Resize to 384x384 (required input for dpt_next_vit_large_384.onnx)
    cv::resize(rgb, rgb, cv::Size(384, 384));

    // Step 3: Convert to float32 and normalize to [0, 1]
    rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

    // Step 4: Mean-std normalization
    std::vector<cv::Mat> channels(3);
    cv::split(rgb, channels);
    channels[0] = (channels[0] - 0.485f) / 0.229f; // R
    channels[1] = (channels[1] - 0.456f) / 0.224f; // G
    channels[2] = (channels[2] - 0.406f) / 0.225f; // B
    cv::merge(channels, rgb);

    // Step 5: Convert to OpenCV DNN blob format
    anOutput = cv::dnn::blobFromImage(rgb); // shape: [1,3,384,384]

    return true;
}

bool MidasModel::postprocess(const cv::Mat& netOutput, std::shared_ptr<IModelOutput>& output)
{
    if (netOutput.empty()) {
        std::cerr << "[ERROR] netOutput is empty in MidasModel::postprocess()\n";
        return false;
    }

    auto depth = std::make_shared<DepthOutput>();

    cv::Mat depthMap;

    if (netOutput.dims == 4 && netOutput.size[1] == 1) {
        // Case: [1,1,H,W]
        int height = netOutput.size[2];
        int width = netOutput.size[3];
        depthMap = cv::Mat(height, width, CV_32F, const_cast<float*>(netOutput.ptr<float>())).clone();
    }
    else if (netOutput.dims == 3 && netOutput.size[0] == 1) {
        // Case: [1,H,W]
        int height = netOutput.size[1];
        int width = netOutput.size[2];
        depthMap = cv::Mat(height, width, CV_32F, const_cast<float*>(netOutput.ptr<float>())).clone();
    }
    else {
        std::cerr << "[ERROR] Unexpected output shape: ";
        for (int i = 0; i < netOutput.dims; ++i)
            std::cerr << netOutput.size[i] << "x";
        std::cerr << std::endl;
        return false;
    }

    // Normalize for visualization or downstream use
    cv::normalize(depthMap, depth->depthMap, 0, 1, cv::NORM_MINMAX);

    output = depth;
    return true;
}

