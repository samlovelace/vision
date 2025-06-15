
#include "ObjectDetectionHandler.h"
#include "ModelFactory.h"
#include "EngineFactory.h"
#include "plog/Log.h"

#include <opencv2/opencv.hpp>

ObjectDetectionHandler::ObjectDetectionHandler(const YAML::Node& aModelsConfig, 
                                               std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aFrameQueue, 
                                               std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue,
                                               std::shared_ptr<ConcurrentQueue<Detection>> aVisQueue, 
                                               std::shared_ptr<InferenceHandler> anInferenceHandler) : 
    mFrameQueue(aFrameQueue), mDetectionQueue(aDetectionQueue), mVisQueue(aVisQueue), mInferenceHandler(anInferenceHandler)
{

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
                for(auto& [objClass, detectionData] : detections->mDetections)
                {
                    LOGD << "Detected class: " << objClass; 
                }
                
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
        }
    }
}



