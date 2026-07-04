#ifndef OBJECTPOSEFILTER_H
#define OBJECTPOSEFILTER_H

#include <opencv2/opencv.hpp>
#include <cstddef>
#include <deque>

// Sliding-window average + variance of a stream of poses. Knows nothing
// about detection algorithms or publishing decisions - callers feed it
// samples and read back the window mean/variance to decide for themselves
// whether the estimate is stable, and whether it has moved meaningfully
// from some reference pose (e.g. the last one they acted on).
class ObjectPoseFilter
{
public:
    explicit ObjectPoseFilter(size_t aWindowSize = 20);

    void update(const cv::Point3f& aPosition, const cv::Vec4f& anOrientation);
    void reset();

    cv::Point3f getMeanPosition() const { return mMeanPosition; }
    cv::Vec4f getMeanOrientation() const { return mMeanOrientation; }

    // per-axis variance of position over the current window
    cv::Point3f getPositionVariance() const;
    // mean of the per-axis variances, for simple single-value thresholding
    float getPositionVarianceMagnitude() const;

    // true if the window mean has moved from aReferencePosition by more
    // than aSignificanceThreshold (euclidean distance)
    bool hasSignificantChange(const cv::Point3f& aReferencePosition, float aSignificanceThreshold) const;

    size_t getSampleCount() const { return mWindow.size(); }

private:
    struct Sample
    {
        cv::Point3f position;
        cv::Vec4f orientation;
    };

    void recomputeMean();

private:
    size_t mWindowSize;
    std::deque<Sample> mWindow;

    cv::Point3f mMeanPosition;
    cv::Vec4f mMeanOrientation; // wxyz, kept normalized
};

#endif //OBJECTPOSEFILTER_H
