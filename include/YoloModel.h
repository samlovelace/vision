#ifndef YOLOMODEL_H
#define YOLOMODEL_H
 
#include "IModel.hpp"
 
class YoloModel : public IModel
{ 
public:
    YoloModel();
    ~YoloModel();

    bool init() override; 
    bool preprocess() override; 
    bool postprocess() override; 

private:
   
};
#endif //YOLOMODEL_H