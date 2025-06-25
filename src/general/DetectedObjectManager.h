#ifndef DETECTEDOBJECTMANAGER_H
#define DETECTEDOBJECTMANAGER_H
 
#include "DetectedObject.hpp"
#include "Detection.hpp"

#include <map>  

class DetectedObjectManager 
{ 
public:
    DetectedObjectManager();
    ~DetectedObjectManager();

    void storeObject(const Detection2D& aDetection, cv::Point3f aCentroid_G); 
    std::vector<DetectedObject> getObjects(const std::string& anObjectType); 

private:
    
    // key: instance_id
    std::map<std::string, DetectedObject> mObjects; 

    std::string generateInstanceId(const std::string& aClassName); 
   
};
#endif //DETECTEDOBJECTMANAGER_H