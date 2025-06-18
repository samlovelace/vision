
#include "ObjectDetectionHandler.h"
#include "ModelFactory.h"
#include "EngineFactory.h"
#include "plog/Log.h"
#include "Utils.hpp"

#include <opencv2/opencv.hpp>

ObjectDetectionHandler::ObjectDetectionHandler(const YAML::Node& aDetectionConfig, 
                                               std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aFrameQueue, 
                                               std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue,
                                               std::shared_ptr<ConcurrentQueue<Detection>> aVisQueue, 
                                               std::shared_ptr<InferenceHandler> anInferenceHandler) : 
    mFrameQueue(aFrameQueue), mDetectionQueue(aDetectionQueue), mVisQueue(aVisQueue), mInferenceHandler(anInferenceHandler), 
    mMinConfidenceThreshold(0.5), mSimilarDetectionThreshold(0.75)
{
    mMinConfidenceThreshold = aDetectionConfig["min_confidence"].as<float>(); 
    mSimilarDetectionThreshold = aDetectionConfig["similar_detection"].as<float>(); 

}

ObjectDetectionHandler::~ObjectDetectionHandler()
{

}

void ObjectDetectionHandler::run()
{
    while(true)
    {
        StampedCameraOutput frame; 
        if(mFrameQueue->pop(frame))
        { 
            Detection detection;

            // TODO: could make this configurable so that we could do 2d-detection or mask-segementation to find objects ?
            auto output = mInferenceHandler->runInference("2d-detection", frame.frames.left.mFrame); 

            if(nullptr != output)
            {
                auto detections = std::dynamic_pointer_cast<DetectionOutput>(output);

                // TODO: loop through all detections for each type and consolidate if bbox centroid within some threshold? 
                removeLowConfidenceDetections(detections); 
                removeSimilarDetections(detections); 

                detection.mDetections = detections; 
                detection.mCameraOutput = frame;
                
                int width = frame.mParams->mImgSize.first; 
                int height = frame.mParams->mImgSize.second; 
                cv::resize(detection.mCameraOutput.frames.left.mFrame, detection.mCameraOutput.frames.left.mFrame, cv::Size(width, height)); 

                // push to queue for 3d estimator
                mDetectionQueue->push(detection); 
            }

            if(nullptr != mVisQueue)
            {
                // TODO: maybe there is a better way to clone this frame so the bounding box doesnt show in the image processes by the depth estimation
                Detection detectionToRender; 
                detectionToRender.mDetections = detection.mDetections; 
                detectionToRender.mCameraOutput.frames.left.mFrame = detection.mCameraOutput.frames.left.mFrame.clone(); 

                // get model input size to scale bounding box rendering 
                std::pair<int, int> imgSize = mInferenceHandler->getModelInputSize("2d-detection");

                renderDetections(detectionToRender, imgSize.first, imgSize.second); 
                mVisQueue->push(detectionToRender);
            }
        }
    }
}

void ObjectDetectionHandler::renderDetections(Detection& aDetection, const int aModelInputWidth, const int aModelInputHeight)
{
    // Actual image size (resized back to original resolution)
    int imageWidth = aDetection.mCameraOutput.frames.left.mFrame.cols;
    int imageHeight = aDetection.mCameraOutput.frames.left.mFrame.rows;

    // Compute scaling factors
    float scaleX = static_cast<float>(imageWidth) / aModelInputWidth;
    float scaleY = static_cast<float>(imageHeight) / aModelInputHeight;

    for (const auto& [type, detections] : aDetection.mDetections->mDetections)
    {
        for (const auto& inst : detections)
        {
            // Scale the bounding box to match original image size
            cv::Rect scaledBox;
            scaledBox.x = static_cast<int>(inst.bounding_box.x * scaleX);
            scaledBox.y = static_cast<int>(inst.bounding_box.y * scaleY);
            scaledBox.width = static_cast<int>(inst.bounding_box.width * scaleX);
            scaledBox.height = static_cast<int>(inst.bounding_box.height * scaleY);

            // Draw the bounding box
            cv::rectangle(aDetection.mCameraOutput.frames.left.mFrame, scaledBox, cv::Scalar(0, 255, 0), 1, 8);

            // Prepare label text
            std::ostringstream labelStream;
            labelStream << inst.class_name << " " << std::fixed << std::setprecision(2) << inst.confidence;
            std::string label = labelStream.str();

            int baseLine = 0;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

            // Ensure the label doesn't go above the image
            int top = std::max(scaledBox.y, labelSize.height + 4);

            // Draw background rectangle for the label
            cv::rectangle(aDetection.mCameraOutput.frames.left.mFrame,
                          cv::Point(scaledBox.x, top - labelSize.height - 4),
                          cv::Point(scaledBox.x + labelSize.width, top + baseLine - 4),
                          cv::Scalar(0, 255, 0),
                          cv::FILLED);

            // Draw label text
            cv::putText(aDetection.mCameraOutput.frames.left.mFrame, label,
                        cv::Point(scaledBox.x, top - 2),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }
    }
}


void ObjectDetectionHandler::removeLowConfidenceDetections(std::shared_ptr<DetectionOutput>& aDetections)
{
    for (auto mapIt = aDetections->mDetections.begin(); mapIt != aDetections->mDetections.end(); ) 
    {
        auto& detectionData = mapIt->second;

        for (auto it = detectionData.begin(); it != detectionData.end(); ) 
        {
            if (it->confidence < mMinConfidenceThreshold)
                it = detectionData.erase(it);
            else
                ++it;
        }

        if (detectionData.empty())
            mapIt = aDetections->mDetections.erase(mapIt);
        else
            ++mapIt;
    }

}

void ObjectDetectionHandler::removeSimilarDetections(std::shared_ptr<DetectionOutput>& aDetections)
{
    for (auto& [objClass, detections] : aDetections->mDetections)
    {
        if (detections.size() <= 1)
            continue;

        // Loop through all pairs and remove lower-confidence duplicates
        for (size_t i = 0; i < detections.size(); ++i) 
        {
            for (size_t j = i + 1; j < detections.size(); ) 
            {
                float iou = Utils::computeIoU(detections[i].bounding_box, detections[j].bounding_box);
                if (iou > mSimilarDetectionThreshold) 
                {
                    // Keep the one with higher confidence
                    if (detections[i].confidence >= detections[j].confidence)
                    {
                        detections.erase(detections.begin() + j);
                    }
                    else 
                    {
                        detections.erase(detections.begin() + i);
                        --i; // restart this i to compare remaining j's
                        break;
                    }
                } 
                else 
                {
                    ++j;
                }
            }
        }
    }
}



