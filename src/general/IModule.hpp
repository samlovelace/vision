#ifndef IMODULE_HPP
#define IMODULE_HPP
 
 
class IModule 
{ 
public:
    virtual ~IModule() = default; 
    virtual void start() = 0; 
    virtual void stop() = 0; 

private:
   
};
#endif //IMODULE_HPP