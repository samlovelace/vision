#ifndef IMODEL_H
#define IMODEL_H
 
 
class IModel 
{ 
public:
    ~IModel() = default; 
    
    virtual bool init() = 0; 
    virtual bool preprocess() = 0; 
    virtual bool postprocess() = 0; 

private:
   
};
#endif //IMODEL_H