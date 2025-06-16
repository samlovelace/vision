
#include "ObjectDetectionHandler.h"
#include "ModelFactory.h"
#include "EngineFactory.h"
#include "plog/Log.h"

#include <opencv2/opencv.hpp>

ObjectDetectionHandler::ObjectDetectionHandler(const YAML::Node& aDetectionConfig, 
                                               std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aFrameQueue, 
                                               std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue,
                                               std::shared_ptr<ConcurrentQueue<Detection>> aVisQueue, 
                                               std::shared_ptr<InferenceHandler> anInferenceHandler) : 
    mFrameQueue(aFrameQueue), mDetectionQueue(aDetectionQueue), mVisQueue(aVisQueue), mInferenceHandler(anInferenceHandler), 
    mMinConfidenceThreshold(0.5)
{
    mMinConfidenceThreshold = aDetectionConfig["min_confidence"].as<float>(); 

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

                
                detection.mDetections = detections; 
                detection.mCameraOutput = frame;  

                // push to queues for 3d estimator
                mDetectionQueue->push(detection); 
            }

            if(nullptr != mVisQueue)
            {
                renderDetections(detection); 
                mVisQueue->push(detection);
            }
        }
    }
}

void ObjectDetectionHandler::renderDetections(Detection& aDetection)
{
    for(const auto& [type, detections] : aDetection.mDetections->mDetections)
    {
        for(const auto& inst : detections)
        {
            cv::rectangle(aDetection.mCameraOutput.frames.left.mFrame, inst.bounding_box, cv::Scalar(0, 255, 0, 0), 1, 8); 
            // Prepare label text
            std::ostringstream labelStream;
            labelStream << inst.class_name << " " << std::fixed << std::setprecision(2) << inst.confidence;
            std::string label = labelStream.str();

            int baseLine = 0;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

            // Make sure label doesn't go outside the image
            int top = std::max(inst.bounding_box.y, labelSize.height + 4);

            // Draw background rectangle for the label
            cv::rectangle(aDetection.mCameraOutput.frames.left.mFrame,
                cv::Point(inst.bounding_box.x, top - labelSize.height - 4),
                cv::Point(inst.bounding_box.x + labelSize.width, top + baseLine - 4),
                cv::Scalar(0, 255, 0, 0),
                cv::FILLED);

            // Draw label text
            cv::putText(aDetection.mCameraOutput.frames.left.mFrame, label,
                cv::Point(inst.bounding_box.x, top - 2),
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



