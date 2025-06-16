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
        auto detections = std::make_shared<DetectionOutput>();

        // Format: [1, 1, N, 7]
        const int numDetections = netOutput.size[2];
        const float* data = reinterpret_cast<float*>(netOutput.data);

        for (int i = 0; i < numDetections; ++i) 
        {
            int classId = static_cast<int>(data[i * 7 + 1]);

            int xLeft   = static_cast<int>(data[i * 7 + 3] * imageWidth);
            int yTop    = static_cast<int>(data[i * 7 + 4] * imageHeight);
            int xRight  = static_cast<int>(data[i * 7 + 5] * imageWidth);
            int yBottom = static_cast<int>(data[i * 7 + 6] * imageHeight);

            cv::Rect bbox(cv::Point(xLeft, yTop), cv::Point(xRight, yBottom));

            Detection2D det;
            det.class_name = getClassName(classId);
            det.confidence = data[i * 7 + 2];
            det.bounding_box = bbox;
            det.segmentation_mask = cv::Mat(); // Not available for MobileNet-SSD

            detections->mDetections[det.class_name].emplace_back(det);
        }

        output = detections;
        return true;
    }

    std::string getClassName(int id) {
        if (id >= 0 && id < classNames.size())
            return classNames[id];
        return "unknown";
    }

    void setInputSize(int w, int h) {
        imageWidth = w;
        imageHeight = h;
    }

private:
    int imageWidth = 640;
    int imageHeight = 480;

    const std::vector<std::string> classNames = {
    "background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair",
    "cow", "diningtable", "dog", "horse",
    "motorbike", "person", "pottedplant",
    "sheep", "sofa", "train", "tvmonitor"
    };
};
