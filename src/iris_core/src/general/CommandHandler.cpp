
#include "CommandHandler.h"
#include "iris_common/RosTopicManager.hpp"
#include "plog/Log.h"

CommandHandler::CommandHandler(std::shared_ptr<Vision> aVisionPtr) 
    : mVision(aVisionPtr)
{

}

CommandHandler::~CommandHandler()
{

}

void CommandHandler::init()
{
    RosTopicManager::getInstance()->createSubscriber<ptera_msgs::msg::VisionCommand>("/vision/command", 
        std::bind(&CommandHandler::commandCallback, this, std::placeholders::_1)); 
    
    RosTopicManager::getInstance()->spinNode(); 
}

void CommandHandler::commandCallback(ptera_msgs::msg::VisionCommand::SharedPtr aCommand)
{
    if("find_object" == aCommand->command.data)
    {
        mVision->dispatch(aCommand->object_type.data); 
    }
    else if("disable" == aCommand->command.data)
    {
        mVision->stop(); 
    }
}
