#ifndef IINFERENCEENGINE_H
#define IINFERENCEENGINE_H
 
 
class IInferenceEngine 
{ 
public:
    ~IInferenceEngine() = default; 

    virtual bool init() = 0; 

private:
   
};
#endif //IINFERENCEENGINE_H