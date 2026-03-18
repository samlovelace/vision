#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ArucoBoardDetector.h"
#include "plog/Log.h"

ArucoBoardDetector::ArucoBoardDetector(const YAML::Node& aConfig, 
    std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aFrameQueue, 
    std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aVisQueue) : 
        mFrameQueue(aFrameQueue), 
        mVisQueue(aVisQueue)
{  
    LOGV << YAML::Dump(aConfig); 
    mType = aConfig["board"]["type"].as<std::string>(); 
    std::string boardFilePath = aConfig["board"]["file"].as<std::string>(); 

    std::string packagePath = ament_index_cpp::get_package_share_directory("vision");
    std::string configPath = packagePath + "/configuration/";

    if(!parseBoardFile(configPath + boardFilePath))
    {
        LOGE << "Failed to parse board file at " << boardFilePath; 
        return; // TODO: respond to commander in some way 
    }
    
    

}

ArucoBoardDetector::~ArucoBoardDetector()
{

}

bool ArucoBoardDetector::parseBoardFile(const std::string& aBoardFile)
{
    YAML::Node boardConfig = YAML::LoadFile(aBoardFile);\
    std::vector<std::vector<cv::Point3f>> objPoints;
    std::vector<int> ids;

    for(const auto& tag : boardConfig["tags"])
    {
        ids.push_back(tag["id"].as<int>());

        std::vector<cv::Point3f> corners;
        for(const auto& cornerLoc : tag["corners"])
        {
            auto xyz = cornerLoc.as<std::vector<float>>();
            corners.emplace_back(xyz[0], xyz[1], xyz[2]);
        }
        objPoints.push_back(corners);
    }

    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    mBoard = std::make_shared<cv::aruco::Board>(objPoints, dictionary, ids);

    return true;
}

bool ArucoBoardDetector::parseDictionaryType(const YAML::Node& aConfig, cv::aruco::PredefinedDictionaryType& aDictionaryType)
{
    std::string dictionaryType = aConfig["dictionary"].as<std::string>(); 

    if("DICT_APRILTAG_36h11" == dictionaryType)
    {
        aDictionaryType = cv::aruco::PredefinedDictionaryType::DICT_APRILTAG_36h11; 
    }
    else
    {
        LOGE << "Unsupported dictionary type" << dictionaryType; 
        return false; 
    }

    return true; 
}

void ArucoBoardDetector::run()
{
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    cv::aruco::DetectorParameters detectorParams; 
    cv::aruco::ArucoDetector detector(mBoard->getDictionary(), detectorParams); 

    std::vector<int> markerIds; 
    std::vector<std::vector<cv::Point2f>> markerCorners, rejected;
    cv::Vec3d rvec, tvec; 

    StampedCameraOutput output;
    setRunning(true); 
    LOGV << "Starting Board Detector Loop!"; 

    while(isRunning())
    {
        if(mFrameQueue->pop(output))
        {
            LOGV << "Looking for tags in new frame"; 

            cv::Mat frame = output.frames.left.mFrame;
            auto cameraMatrix = output.mParams->mIntrinsics->cvCameraMatrix();  

            detector.detectMarkers(frame, markerCorners, markerIds, rejected); 
            LOGV << "Detected " << markerIds.size() << " markers. Rejected " << rejected.size(); 

            if(!markerIds.empty())
            {
                LOGV << "Refining detection..."; 
                cv::aruco::refineDetectedMarkers(frame, mBoard, markerCorners, markerIds, 
                                                 rejected, cameraMatrix, distCoeffs);

                LOGV << "Estimating board pose..."; 
                int markersDetected = cv::aruco::estimatePoseBoard(markerCorners, markerIds, mBoard,
                                                                    cameraMatrix, distCoeffs, rvec, tvec); 

                if(markersDetected > 0)
                {
                    LOGV << "Refined detection to " << markersDetected << " markers...";
                    
                    cv::Matx33d R;
                    cv::Rodrigues(rvec, R);
                    cv::Vec3d rpy(atan2(R(2,1), R(2,2)),
                                  atan2(-R(2,0), sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2))),
                                  atan2(R(1,0), R(0,0)));
                    LOGV << "xyz (m): " << tvec << " rpy (deg): " << rpy * 180.0/M_PI;

                    // compute [G]lobal pose of [O]bject based on pose in [C]amera frame 
                    cv::Matx44f T_C_O = cv::Affine3d(rvec, tvec).matrix;
                    cv::Matx44f T_G_O = output.T_G_C * T_C_O; 

                    

                    // draw axes on board origin
                    if(mVisQueue) 
                    {
                        cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);    
                        cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, 0.1f); 
                        mVisQueue->push(output);
                    }     
                }
            }
        }        
    }
}

