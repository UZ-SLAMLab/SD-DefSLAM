/*
 * This file is part of SD-DefSLAM
 * Copyright (C) 2020 Juan J. Gómez Rodríguez, Jose Lamarca Peiro, J. Morlana,
 *                    Juan D. Tardós and J.M.M. Montiel, University of Zaragoza
 *
 * This software is for internal use in the EndoMapper project.
 * Not to be re-distributed.
 */

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