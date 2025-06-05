#include "IModel.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class MobileNetSSDModel : public IModel
{
public:
    bool init() override 
    {
        return true; 
    }

    bool preprocess(const cv::Mat& inputImage, cv::Mat& outputBlob) override 
    {
        // MobileNet-SSD expects 300x300, BGR, mean subtraction, and scaling
        outputBlob = cv::dnn::blobFromImage(inputImage,
                                            0.007843, // scale factor
                                            cv::Size(300, 300),
                                            cv::Scalar(127.5, 127.5, 127.5),
                                            false,    // BGR order (no swap)
                                            false     // no crop
                                        );

        return true;
    }

    bool postprocess(const cv::Mat& netOutput, std::shared_ptr<IModelOutput>& output) override 
    {
        std::vector<cv::Rect> boxes; 
        auto detections = std::make_shared<DetectionOutput>(); 
        const float confidenceThreshold = 0.15;

        // Format: [1, 1, N, 7]
        for (int i = 0; i < netOutput.size[2]; ++i) {
            float confidence = netOutput.ptr<float>(0, 0, i)[2];
            if (confidence > confidenceThreshold) {
                int xLeft = static_cast<int>(netOutput.ptr<float>(0, 0, i)[3] * imageWidth);
                int yTop  = static_cast<int>(netOutput.ptr<float>(0, 0, i)[4] * imageHeight);
                int xRight = static_cast<int>(netOutput.ptr<float>(0, 0, i)[5] * imageWidth);
                int yBottom = static_cast<int>(netOutput.ptr<float>(0, 0, i)[6] * imageHeight);
                boxes.emplace_back(cv::Rect(cv::Point(xLeft, yTop), cv::Point(xRight, yBottom)));
            }
        }

        detections->boxes = boxes; 
        output = detections; 
        return true;
    }

    void setInputSize(int w, int h) {
        imageWidth = w;
        imageHeight = h;
    }

private:
    cv::dnn::Net net;
    std::string protoPath = "/home/sam/models/deploy.prototxt";
    std::string modelPath = "/home/sam/models/mobilenet.caffemodel";
    int imageWidth = 300;
    int imageHeight = 300;

    const std::vector<std::string> classNames = {
    "background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair",
    "cow", "diningtable", "dog", "horse",
    "motorbike", "person", "pottedplant",
    "sheep", "sofa", "train", "tvmonitor"
    };
};
