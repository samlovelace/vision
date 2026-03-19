#ifndef ARUCOBOARDDETECTOR_H
#define ARUCOBOARDDETECTOR_H

#include <yaml-cpp/yaml.h>
 
#include <opencv2/objdetect/aruco_detector.hpp>    // ArucoDetector, DetectorParameters
#include <opencv2/objdetect/aruco_board.hpp>       // Board, GridBoard
#include <opencv2/aruco.hpp>                       // estimatePoseBoard, drawAxis
#include <opencv2/calib3d.hpp>                     // Rodrigues (if converting rvec to rotation matrix)
#include <opencv2/core.hpp>                        // Mat, Vec3d, Point3f etc
#include <opencv2/imgproc.hpp>                     // if doing any frame preprocessing
 
#include "ConcurrentQueue.hpp"
#include "CameraData.hpp"

class ArucoBoardDetector 
{ 
public:
    ArucoBoardDetector(const YAML::Node& aConfig, 
                       std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aFrameQueue,
                       std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> aVisQueue);
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
};
#endif //ARUCOBOARDDETECTOR_H