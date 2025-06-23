
#include "ZoeDepthModel.h"
#include <iostream>

ZoeDepthModel::ZoeDepthModel()  {}
ZoeDepthModel::~ZoeDepthModel() {}

bool ZoeDepthModel::init() 
{
    return true;
}

bool ZoeDepthModel::preprocess(const cv::Mat& aFrame, cv::Mat& anOutput)
{
    cv::Mat rgb;
    cv::cvtColor(aFrame, rgb, cv::COLOR_BGR2RGB);
    cv::resize(rgb, rgb, cv::Size(384, 384));
    rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

    // ZoeDepth uses same normalization as MiDaS (ImageNet)
    std::vector<cv::Mat> channels(3);
    cv::split(rgb, channels);
    channels[0] = (channels[0] - 0.485f) / 0.229f;
    channels[1] = (channels[1] - 0.456f) / 0.224f;
    channels[2] = (channels[2] - 0.406f) / 0.225f;
    cv::merge(channels, rgb);

    anOutput = cv::dnn::blobFromImage(rgb); // [1, 3, 384, 384]
    return true;
}

bool ZoeDepthModel::postprocess(const cv::Mat& netOutput, std::shared_ptr<IModelOutput>& output)
{
    if (netOutput.empty()) {
        std::cerr << "[ERROR] netOutput is empty in ZoeDepthModel::postprocess()\n";
        return false;
    }

    auto depth = std::make_shared<DepthOutput>();
    cv::Mat depthMap;

    if (netOutput.dims == 4 && netOutput.size[1] == 1) {
        int height = netOutput.size[2];
        int width = netOutput.size[3];
        depthMap = cv::Mat(height, width, CV_32F, const_cast<float*>(netOutput.ptr<float>())).clone();
    }
    else {
        std::cerr << "[ERROR] Unexpected output shape: ";
        for (int i = 0; i < netOutput.dims; ++i)
            std::cerr << netOutput.size[i] << "x";
        std::cerr << std::endl;
        return false;
    }

    // No normalization here: ZoeDepth outputs real depth in meters
    depth->depthMap = depthMap;  // preserve raw metric values

    output = depth;
    return true;
}
