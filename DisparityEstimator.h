
#ifndef __DISPARITYESTIMATOR_H__
#define __DISPARITYESTIMATOR_H__

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace libelas
{
    class DisparityEstimator
    {
    public:
        // Initialize the class
        DisparityEstimator() = default;

        // Process the disparity of an image
        cv::Mat process(const cv::Mat &, const cv::Mat &);
    };
} // namespace libelas
#endif