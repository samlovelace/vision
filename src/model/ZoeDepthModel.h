#ifndef ZOEDEPTHMODEL_H
#define ZOEDEPTHMODEL_H
 
#include "IModel.hpp"
 
class ZoeDepthModel : public IModel
{ 
public:
    ZoeDepthModel();
    ~ZoeDepthModel();

    bool init() override; 
    bool preprocess(const cv::Mat& inputImage, cv::Mat& outputBlob) override; 
    bool postprocess(const cv::Mat& netOutput, std::shared_ptr<IModelOutput>& output) override; 


private:
   
};
#endif //ZOEDEPTHMODEL_HPP