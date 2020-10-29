/*
 * This file is part of SD-DefSLAM
 * Copyright (C) 2020 Juan J. Gómez Rodríguez, Jose Lamarca Peiro, J. Morlana,
 *                    Juan D. Tardós and J.M.M. Montiel, University of Zaragoza
 *
 * This software is for internal use in the EndoMapper project.
 * Not to be re-distributed.
 */

#include <opencv2/opencv.hpp>

#include "BorderMask.h"
#include <iostream>

namespace defSLAM
{
    cv::Mat BorderMask::generateMask(const cv::Mat &im)
    {
        cv::Mat imGray = im, mask;

        if (imGray.channels() == 3)
        {
            cvtColor(imGray, imGray, cv::COLOR_BGR2GRAY);
        }
        else if (imGray.channels() == 4)
        {
            cvtColor(imGray, imGray, cv::COLOR_BGR2GRAY);
        }
        cv::Rect roi(cb_, rb_, imGray.cols - ce_ - cb_, imGray.rows - re_ - rb_);
        cv::Mat maska = cv::Mat::zeros(imGray.size(), CV_8U);
        maska(roi) = cv::Scalar(255);
        maska.setTo(0, imGray == 0);
        cv::erode(maska, maska, getStructuringElement(cv::MORPH_RECT, cv::Size(21, 21)));
        return maska;
    }

    std::string BorderMask::getDescription()
    {
        return std::string("Border mask with parameters [" + std::to_string(rb_) + "," + std::to_string(re_) + "," +
                           std::to_string(cb_) + "," + std::to_string(ce_) + "]");
    }
} // namespace defSLAM