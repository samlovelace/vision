
#include "PoseEstimationHandler.h"
#include "MonocularDepthEstimator.h"
#include <thread> 

PoseEstimationHandler::PoseEstimationHandler(const YAML::Node& aPoseEstConfig, 
                                             std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue, 
                                             std::shared_ptr<InferenceHandler> anInferenceHandler, 
                                             std::shared_ptr<ConcurrentQueue<cv::Mat>> aDepthMapVisQueue, 
                                             std::shared_ptr<ConcurrentQueue<pcl::PointCloud<pcl::PointXYZ>::Ptr>> aPcVisQueue) : 
    mDetectionQueue(aDetectionQueue), mDepthMapVisQueue(aDepthMapVisQueue), mCloudVisQueue(aPcVisQueue), mObjCloudGenerator(std::make_shared<ObjectCloudGenerator>())
{
    std::string depthEstType = aPoseEstConfig["depth_estimation"].as<std::string>(); 

    if("monocular" == depthEstType)
    {
        if(!anInferenceHandler->hasModelType("depth"))
        {
            throw std::runtime_error("Cannot use monocular depth estimation without depth estimating neural network"); 
        }
        mDepthEstimator = std::make_shared<MonocularDepthEstimator>(anInferenceHandler); 
    }
    else
    {
        LOGE << "Unsupported depth estimator of type: " << depthEstType; 
        mDepthEstimator = nullptr; 
    }
    
    if(nullptr == mDepthEstimator)
    {
        throw std::runtime_error("Invalid pose estimation configuration"); 
    }
}

PoseEstimationHandler::~PoseEstimationHandler()
{

}

void PoseEstimationHandler::run()
{
    while(true)
    {
        Detection detection; 

        if(mDetectionQueue->pop(detection))
        {
            cv::Mat depthMap; 

            if(!mDepthEstimator->estimateDepth(detection.mCameraOutput.frames, depthMap))
            {
                LOGW << "Could not compute depth map!"; 
                continue; 
            }

            std::pair<int, int> imgSize = detection.mCameraOutput.mParams->mImgSize; 
            cv::resize(depthMap, depthMap, cv::Size(imgSize.first, imgSize.second)); 
            mDepthMapVisQueue->push(depthMap); 

            for(auto& [obj, detections] : detection.mDetections->mDetections)
            {
                for(auto& det : detections)
                {
                    // TODO: need some way of keeping track of unique objects of same type
                    // Probably best to use the objectLibrary once that is implemented
                    auto cloud = mObjCloudGenerator->generateCloud(det.bounding_box, depthMap, detection.mCameraOutput.mParams); 

                    mCloudVisQueue->push(cloud); 

                }
            }
              
        }
    }
}

