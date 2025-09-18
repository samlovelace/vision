
#include "DetectedObjectManager.h"
#include <numeric>
#include "plog/Log.h"
#include <opencv2/core.hpp>
#include <cmath>

DetectedObjectManager::DetectedObjectManager()
{

}

DetectedObjectManager::~DetectedObjectManager()
{

}

void DetectedObjectManager::storeObject(const Detection2D& aDetection, cv::Point3f aCentroid_G)
{   
    std::vector<DetectedObject> objsOfSameClass = getObjects(aDetection.class_name); 

    // TODO: make config 
    float threshold = 0.3; 

    bool foundSimilar = false; 

    if(!objsOfSameClass.empty())
    {
        for(auto& obj : objsOfSameClass)
        {
            if(cv::norm(obj.global_centroid - aCentroid_G) < threshold)
            {
                // assume same object, update centroid and confidence to avg 
                obj.global_centroid = (obj.global_centroid + aCentroid_G) * 0.5f; 
                obj.confidence = (obj.confidence + aDetection.confidence) * 0.5f;  
                obj.last_seen = std::chrono::system_clock::now();
                
                // break because we dont need to check any more objects 
                foundSimilar = true; 
                break;  
            }
        }
    }

    if(!foundSimilar)
    {
        LOGV << "Adding new detected object of type: " << aDetection.class_name; 
        addNewObject(aDetection, aCentroid_G); 
    } 
}

void DetectedObjectManager::addNewObject(const Detection2D& aDetection, cv::Point3f aCentroid_G)
{
    // create new detect object  
    DetectedObject obj; 
    obj.class_label = aDetection.class_name; 
    obj.confidence = aDetection.confidence; 

    obj.global_centroid = aCentroid_G; 

    // generate unique instance id
    obj.instance_id = generateInstanceId(aDetection.class_name); 

    obj.last_seen = std::chrono::system_clock::now(); 

    mObjects.insert({obj.instance_id, obj}); 
    LOGV << "Detected " << obj.instance_id << " at " << obj.global_centroid;
}

std::string DetectedObjectManager::generateInstanceId(const std::string& aClassName)
{
    LOGV << "Generating new object instance ID"; 
    std::string instanceLabel = aClassName; 
    int maxId = 1000; 
    int num = 0;  

    if(mObjects.empty())
    {   
        instanceLabel += "_" + std::to_string(0); 
        return instanceLabel; 
    }

    for (const auto& obj : getObjects(aClassName)) 
    {
        if (obj.class_label == aClassName) 
        {
            // Expecting id format like "cat_001"
            std::string suffix = obj.instance_id.substr(aClassName.size() + 1); // +1 for '_'
            LOGV << "Suffix: " << suffix; 
            
            try {
                    num = std::stoi(suffix);
                    num++; // increment

                    if (num > maxId) 
                    {
                        num = maxId; 
                    }
            } catch (...) {
                // skip malformed IDs
            }
        }
    }
 
    return instanceLabel + "_" + std::to_string(num); 
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



