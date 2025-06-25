
#include "DetectedObjectManager.h"
#include <numeric>
#include "plog/Log.h"

DetectedObjectManager::DetectedObjectManager()
{

}

DetectedObjectManager::~DetectedObjectManager()
{

}

void DetectedObjectManager::storeObject(const Detection2D& aDetection, cv::Point3f aCentroid_G)
{
    DetectedObject obj; 
    obj.class_label = aDetection.class_name; 
    obj.confidence = aDetection.confidence; 

    obj.global_centroid = aCentroid_G; 

    // TODO: check if object is similar in global position to other object of same type,
    // if within threshold assume same object and update global centroid 

    // generate unique instance id
    obj.instance_id = generateInstanceId(aDetection.class_name); 

    obj.last_seen = std::chrono::system_clock::now(); 

    mObjects.insert({obj.instance_id, obj}); 
    LOGV << "Detected " << obj.instance_id << " at " << obj.global_centroid; 
}

std::string DetectedObjectManager::generateInstanceId(const std::string& aClassName)
{
    std::string instanceLabel = aClassName; 
    int maxId = 0; 

    if(mObjects.empty())
    {   
        instanceLabel += "_" + std::to_string(0); 
        return instanceLabel; 
    }

    for (const auto& [id, obj] : mObjects) 
    {
        if (obj.class_label == aClassName) 
        {
            // Expecting id format like "cat_001"
            std::string suffix = id.substr(aClassName.size() + 1); // +1 for '_'
            
            try {
                    int num = std::stoi(suffix);
                    if (num > maxId) 
                    {
                        maxId = num;
                    }
            } catch (...) {
                // skip malformed IDs
            }
        }
    }

    return instanceLabel + "_" + std::to_string(maxId); 
}

std::vector<DetectedObject> DetectedObjectManager::getObjects(const std::string& anObjectType)
{
    std::vector<DetectedObject> objs; 

    for(const auto& [id, obj] : mObjects)
    {
        if(obj.class_label == anObjectType)
        {
            objs.push_back(obj); 
        }
    }

    return objs; 
}



