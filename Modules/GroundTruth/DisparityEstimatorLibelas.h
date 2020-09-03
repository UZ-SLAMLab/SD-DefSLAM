
#ifndef __DISPARITYLIBESTIMATOR_H__
#define __DISPARITYLIBESTIMATOR_H__

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace stereodisparity
{
    class DisparityEstimatorLibelas
    {
    public:
        // Initialize the class
        DisparityEstimatorLibelas() = default;

        // Process the disparity of an image
        cv::Mat process(const cv::Mat &, const cv::Mat &);
    };
} // namespace stereodisparity
#endif