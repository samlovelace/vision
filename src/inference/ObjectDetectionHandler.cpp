
#include "ObjectDetectionHandler.h"
#include "ModelFactory.h"
#include "EngineFactory.h"
#include "plog/Log.h"

#include <opencv2/opencv.hpp>

ObjectDetectionHandler::ObjectDetectionHandler(const YAML::Node& aModelsConfig, 
                           std::shared_ptr<ConcurrentQueue<cv::Mat>> aFrameQueue, 
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
        cv::Mat frame; 
        if(mFrameQueue->pop(frame))
        { 
            Detection detection;

            auto output = mInferenceHandler->runInference("2d-detection", frame); 

            if(nullptr != output)
            {
                auto detections = std::dynamic_pointer_cast<DetectionOutput>(output);
                
                detection.mDetections = detections->boxes; 
                detection.mFrame = frame; 

                // push to queues for 3d estimator
                mDetectionQueue->push(detection); 
            }

            // TODO: only render and pus to vis queue if visualizing
            renderDetections(detection); 
            mVisQueue->push(detection); 
        }
    }
}

void ObjectDetectionHandler::renderDetections(Detection& aDetection)
{
    for(const auto& bbox : aDetection.mDetections)
    {
        cv::rectangle(aDetection.mFrame, bbox, cv::Scalar(0, 255, 0, 0), 1, 8); 
    }
}



