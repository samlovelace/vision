#ifndef BOARDLOCATORMODULE
#define BOARDLOCATORMODULE
 
#include <string> 
#include <memory> 
#include <vector>

#include "IModule.hpp"
#include "Types.hpp"
#include "CameraHandler.h"
#include "ConcurrentQueue.hpp" 
#include "CameraData.hpp"
#include "NavDataHandler.h"
#include "DetectedObjectManager.h"
#include "ArucoBoardDetector.h"

class BoardLocatorModule : public IModule 
{ 
public:
    BoardLocatorModule(const KnownObjectConfig& anObjectToFind);
    ~BoardLocatorModule() override; 

    void start() override; 
    void stop() override; 

    bool isRunning() {std::lock_guard<std::mutex> lock(mRunningMutex); return mRunning; }
    void setRunning(bool aFlag) {std::lock_guard<std::mutex> lock(mRunningMutex); mRunning = aFlag; }

private: 
    void run();
    void runVisualizer(); 

private:
    std::vector<std::thread> mThreads; 
    std::shared_ptr<CameraHandler> mCameraHandler; 
    std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> mFrameQueue; 
    std::shared_ptr<NavDataHandler> mNavDataHandler; 
    std::shared_ptr<DetectedObjectManager> mObjectManager; 

    std::shared_ptr<ArucoBoardDetector> mDetector; 

    std::shared_ptr<ConcurrentQueue<StampedCameraOutput>> m2DVisQueue;

    KnownObjectConfig mObjectToFind; 

    bool mVisualize; 
    bool mRunning; 
    std::mutex mRunningMutex;

};
#endif //BOARDLOCATORMODULE