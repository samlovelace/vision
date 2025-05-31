#ifndef MODELCONTEXT_H
#define MODELCONTEXT_H

#include <memory>

#include "IModel.hpp"
#include "IInferenceEngine.hpp"

struct ModelContext
{
    ModelContext(std::unique_ptr<IModel> aModel, std::unique_ptr<IInferenceEngine> anEngine)
        : mModel(std::move(aModel)), mEngine(std::move(anEngine))
    {

    }

    std::unique_ptr<IModel> mModel; 
    std::unique_ptr<IInferenceEngine> mEngine; 
};

#endif // MODELCONTEXT_H