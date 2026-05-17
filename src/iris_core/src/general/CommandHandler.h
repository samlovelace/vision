#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include "Vision.h"
#include "ptera_msgs/msg/vision_command.hpp" 


class CommandHandler 
{ 
public:
    CommandHandler(std::shared_ptr<Vision> aVisionPtr);
    ~CommandHandler();

    void init(); 
    void commandCallback(ptera_msgs::msg::VisionCommand::SharedPtr aCommand); 
private:
    std::shared_ptr<Vision> mVision; 
   
};
#endif //COMMANDHANDLER_H