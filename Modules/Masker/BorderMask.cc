/**
* This file is part of DefSLAM.
*
* Copyright (C) 2017-2020 Jose Lamarca Peiro <jlamarca at unizar dot es>, J.M.M. Montiel (University
*of Zaragoza) && Shaifali Parashar, Adrien Bartoli (Université Clermont Auvergne)
* For more information see <https://github.com/unizar/DefSLAM>
*
* DefSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DefSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DefSLAM. If not, see <http://www.gnu.org/licenses/>.
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