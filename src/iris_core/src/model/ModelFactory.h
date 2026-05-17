#ifndef MODELFACTORY_H
#define MODELFACTORY_H
 
#include "IModel.hpp" 
#include <string> 
#include <memory>

class ModelFactory 
{ 
public:
    ModelFactory() {}
    ~ModelFactory() {}

    static std::unique_ptr<IModel> create(const std::string& aModelType); 

private:
   
};
#endif //MODELFACTORY_H