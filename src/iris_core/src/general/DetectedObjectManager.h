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

    void storeObject(DetectedObject& aDetection); 
    std::vector<DetectedObject> getObjects(const std::string& anObjectType); 

private:
    std::string generateInstanceId(const std::string& aClassName); 
    void addNewObject(DetectedObject& aDetection); 
   
private: 
    // key: instance_id
    std::map<std::string, DetectedObject> mObjects; 

};
#endif //DETECTEDOBJECTMANAGER_H