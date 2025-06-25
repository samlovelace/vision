
#include "CommandHandler.h"
#include "RosTopicManager.hpp"
#include "plog/Log.h"

CommandHandler::CommandHandler(std::shared_ptr<Vision> aVisionPtr) : mVision(aVisionPtr)
{

}

CommandHandler::~CommandHandler()
{

}

void CommandHandler::init()
{
    RosTopicManager::getInstance()->createSubscriber<vision_idl::msg::Command>("/vision/command", std::bind(&CommandHandler::commandCallback, this, std::placeholders::_1)); 

    RosTopicManager::getInstance()->spinNode(); 
}

void CommandHandler::commandCallback(vision_idl::msg::Command::SharedPtr aCommand)
{

    if("find_object" == aCommand->command.data)
    {
        mVision->find_object(aCommand->object_type.data); 
    }
}
