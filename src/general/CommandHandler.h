#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include "Vision.h"
#include "vision_idl/msg/command.hpp" 


class CommandHandler 
{ 
public:
    CommandHandler(std::shared_ptr<Vision> aVisionPtr);
    ~CommandHandler();

    void init(); 
    void commandCallback(vision_idl::msg::Command::Ptr aCommand); 
private:
    std::shared_ptr<Vision> mVision; 
   
};
#endif //COMMANDHANDLER_H