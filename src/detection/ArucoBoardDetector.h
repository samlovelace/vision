#ifndef ARUCOBOARDDETECTOR_H
#define ARUCOBOARDDETECTOR_H

#include <yaml-cpp/yaml.h>
 
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_board.hpp>       
#include <opencv2/aruco.hpp>                       
#include <opencv2/calib3d.hpp>                     
#include <opencv2/core.hpp>                        
#include <opencv2/imgproc.hpp>                     
 
#include "ConcurrentQueue.hpp"
#include "CameraData.hpp"
#include "DetectedObjectManager.h"
#include "Types.hpp"

class ArucoBoardDetector 
{ 
public:
    ArucoBoardDetector(const KnownObjectConfig& aBoardFilePath, 
                       std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aFrameQueue,
                       std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aVisQueue, 
                       std::shared_ptr<DetectedObjectManager> anObjectManager);
    ~ArucoBoardDetector();

    void run(); 

private:

    bool parseBoardFile(const std::string& aBoardFile); 
    bool parseDictionaryType(const YAML::Node& aConfig, cv::aruco::PredefinedDictionaryType& aDictionaryType);

    bool isRunning() {std::lock_guard<std::mutex> lock(mRunningMutex); return mRunning; }
    void setRunning(bool aFlag) {std::lock_guard<std::mutex> lock(mRunningMutex); mRunning = aFlag; }

    void printXYZandRPY(const cv::Matx44f& T);

private: 
    std::string mType; // the semantic type associated with the board 
    std::shared_ptr<cv::aruco::Board> mBoard; 

    bool mRunning; 
    std::mutex mRunningMutex; 

    std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> mFrameQueue;
    std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> mVisQueue; 
    std::shared_ptr<DetectedObjectManager> mObjectManager; 
    const KnownObjectConfig mKnownObjectCfg; 
};
#endif //ARUCOBOARDDETECTOR_H