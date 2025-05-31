#ifndef OPENCVENGINE_H
#define OPENCVENGINE_H
 
#include "IInferenceEngine.hpp"
 
class OpenCvEngine : public IInferenceEngine
{ 
public:
    OpenCvEngine();
    ~OpenCvEngine();

    bool init() override; 

private:
   
};
#endif //OPENCVENGINE_H