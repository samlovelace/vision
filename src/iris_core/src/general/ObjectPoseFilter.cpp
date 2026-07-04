#include "ObjectPoseFilter.h"

#include <limits>

ObjectPoseFilter::ObjectPoseFilter(size_t aWindowSize)
    : mWindowSize(aWindowSize),
      mMeanPosition(0.0f, 0.0f, 0.0f),
      mMeanOrientation(1.0f, 0.0f, 0.0f, 0.0f)
{
}

void ObjectPoseFilter::update(const cv::Point3f& aPosition, const cv::Vec4f& anOrientation)
{
    cv::Vec4f orientation = anOrientation;

    // quaternions double-cover rotations (q and -q are the same rotation);
    // flip onto the same hemisphere as the current mean before storing so
    // opposite-signed samples don't cancel each other out when averaged
    if (!mWindow.empty() && orientation.dot(mMeanOrientation) < 0.0f)
    {
        orientation = -orientation;
    }

    mWindow.push_back({aPosition, orientation});
    while (mWindow.size() > mWindowSize)
    {
        mWindow.pop_front();
    }

    recomputeMean();
}

void ObjectPoseFilter::recomputeMean()
{
    cv::Point3f positionSum(0.0f, 0.0f, 0.0f);
    cv::Vec4f orientationSum(0.0f, 0.0f, 0.0f, 0.0f);

    for (const auto& sample : mWindow)
    {
        positionSum += sample.position;
        orientationSum += sample.orientation;
    }

    float n = static_cast<float>(mWindow.size());
    mMeanPosition = positionSum * (1.0f / n);

    mMeanOrientation = orientationSum;
    float norm = static_cast<float>(cv::norm(mMeanOrientation));
    if (norm > 1e-6f)
    {
        mMeanOrientation *= (1.0f / norm);
    }
}

cv::Point3f ObjectPoseFilter::getPositionVariance() const
{
    // variance of fewer than 2 samples is undefined - report it as
    // unbounded rather than 0 so a sparsely-filled window can't look converged
    if (mWindow.size() < 2)
    {
        float inf = std::numeric_limits<float>::infinity();
        return cv::Point3f(inf, inf, inf);
    }

    cv::Point3f sumSquaredDev(0.0f, 0.0f, 0.0f);
    for (const auto& sample : mWindow)
    {
        cv::Point3f dev = sample.position - mMeanPosition;
        sumSquaredDev.x += dev.x * dev.x;
        sumSquaredDev.y += dev.y * dev.y;
        sumSquaredDev.z += dev.z * dev.z;
    }

    // Bessel's correction (n-1): population variance (n) is biased low,
    // which matters most exactly when the window is sparsely filled
    float n = static_cast<float>(mWindow.size() - 1);
    return cv::Point3f(sumSquaredDev.x / n, sumSquaredDev.y / n, sumSquaredDev.z / n);
}

float ObjectPoseFilter::getPositionVarianceMagnitude() const
{
    cv::Point3f variance = getPositionVariance();
    return (variance.x + variance.y + variance.z) / 3.0f;
}

bool ObjectPoseFilter::hasSignificantChange(const cv::Point3f& aReferencePosition, float aSignificanceThreshold) const
{
    return cv::norm(mMeanPosition - aReferencePosition) > aSignificanceThreshold;
}

void ObjectPoseFilter::reset()
{
    mWindow.clear();
    mMeanPosition = cv::Point3f(0.0f, 0.0f, 0.0f);
    mMeanOrientation = cv::Vec4f(1.0f, 0.0f, 0.0f, 0.0f);
}
