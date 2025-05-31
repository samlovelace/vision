#ifndef ICAMERA_HPP
#define ICAMERA_HPP
 
#include <opencv2/core/mat.hpp>
 
class ICamera 
{ 
public:

    ~ICamera() = default; 

    virtual bool init() = 0; 
    virtual cv::Mat getFrame() = 0; 

private:
   
};
#endif //ICAMERA_HPP