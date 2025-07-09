
#include "PoseEstimationHandler.h"
#include "MonocularDepthEstimator.h"
#include "StereoDepthEstimator.h"
#include <thread> 

PoseEstimationHandler::PoseEstimationHandler(const YAML::Node& aPoseEstConfig, 
                                             std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue, 
                                             std::shared_ptr<InferenceHandler> anInferenceHandler, 
                                             std::shared_ptr<ConcurrentQueue<cv::Mat>> aDepthMapVisQueue, 
                                             std::shared_ptr<ConcurrentQueue<pcl::PointCloud<pcl::PointXYZ>::Ptr>> aPcVisQueue, 
                                             std::shared_ptr<DetectedObjectManager> anObjManager) : 
    mDetectionQueue(aDetectionQueue), mDepthMapVisQueue(aDepthMapVisQueue), mCloudVisQueue(aPcVisQueue), 
    mObjCloudGenerator(std::make_shared<ObjectCloudGenerator>()), 
    mDetectedObjectManager(anObjManager), mRunning(true)
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
    else if ("stereo" == depthEstType)
    {
        mDepthEstimator = std::make_shared<StereoDepthEstimator>(); 
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

    cv::Matx44f T_S_C(
    -1.0f, 0.0f, 0.0f, 0.0f,
     0.0f, 0.0f, 1.0f, 0.0f,
     0.0f, 1.0f, 0.0f, 0.0f,
     0.0f, 0.0f, 0.0f, 1.0f
    );

    while(isRunning())
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
                    auto cloud_Cam = mObjCloudGenerator->generateCloud(det.bounding_box, depthMap, detection.mCameraOutput.mParams); 

                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_G(new pcl::PointCloud<pcl::PointXYZ>);

                    // transform cloud into sensor frame
                    mObjCloudGenerator->transformCloud(cloud_Cam, cloud_Cam, T_S_C);

                    // transform cloud into global frame
                    mObjCloudGenerator->transformCloud(cloud_Cam, cloud_G, detection.mCameraOutput.T_G_C); 

                    // push to visualization queue
                    mCloudVisQueue->push(cloud_G);

                    // compute object centroid 
                    cv::Point3f objCentroid_G = mObjCloudGenerator->computeCloudCentroid(cloud_G); 

                    // TODO: idk a good name for this function 
                    mDetectedObjectManager->storeObject(det, objCentroid_G);
                }
            }
              
        }
    }

    LOGD << "PoseEstimationHandler processing loop exited"; 
}

