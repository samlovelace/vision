
#include "PoseEstimationHandler.h"
#include "MonocularDepthEstimator.h"
#include <thread> 

PoseEstimationHandler::PoseEstimationHandler(const YAML::Node& aPoseEstConfig, 
                                             std::shared_ptr<ConcurrentQueue<Detection>> aDetectionQueue, 
                                             std::shared_ptr<InferenceHandler> anInferenceHandler, 
                                             std::shared_ptr<ConcurrentQueue<cv::Mat>> aDepthMapVisQueue, 
                                             std::shared_ptr<ConcurrentQueue<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> aPcVisQueue) : 
    mDetectionQueue(aDetectionQueue), mDepthMapVisQueue(aDepthMapVisQueue), mCloudVisQueue(aPcVisQueue)
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

            // save camera intrinsics in local variable for easy use
            float fx = detection.mCameraOutput.mParams->mIntrinsics->focalX(); 
            float fy = detection.mCameraOutput.mParams->mIntrinsics->focalY();
            float cx = detection.mCameraOutput.mParams->mIntrinsics->centerX();
            float cy = detection.mCameraOutput.mParams->mIntrinsics->centerY(); 
            cv::Mat rgb = detection.mCameraOutput.frames.left.mFrame.clone(); 

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            for(auto& [obj, detections] : detection.mDetections->mDetections)
            {
                for(auto& det : detections)
                {
                    for(int x = det.bounding_box.x; x < det.bounding_box.height; x++)
                    {
                        for(int y = det.bounding_box.y; y < det.bounding_box.width; y++)
                        {
                            float d = depthMap.at<float>(y, x); 
                            
                            if(d <= 0.0f || std::isnan(d))
                            {
                                continue; 
                            }

                            float X = (x - cx) * d/fx; 
                            float Y = (y - cy) * d/fy; 
                            float Z = d; 

                            cv::Vec3b color = rgb.at<cv::Vec3b>(y, x);
                            pcl::PointXYZRGB pt;
                            pt.x = X;
                            pt.y = Y;
                            pt.z = Z;
                            pt.r = color[2];
                            pt.g = color[1];
                            pt.b = color[0];

                            cloud->points.push_back(pt);
                        }
                    }
                }
            }

            cloud->width = cloud->points.size();
            cloud->height = 1;
            cloud->is_dense = false;

            mCloudVisQueue->push(cloud); 
              
        }
    }
}

