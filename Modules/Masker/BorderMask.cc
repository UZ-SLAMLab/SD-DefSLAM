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

#include "BorderMask.h"
#include <iostream>

namespace defSLAM {
    cv::Mat BorderMask::generateMask(const cv::Mat &im) {
        cv::Mat mask(im.rows, im.cols, CV_8U, cv::Scalar(0));
        cv::Rect roi(cb_,rb_,mask.cols-ce_-cb_,mask.rows-re_-rb_);
        mask(roi) = cv::Scalar(255);

        return mask;
    }

    std::string BorderMask::getDescription() {
        return std::string("Border mask with parameters [" + std::to_string(rb_) + "," + std::to_string(re_) + "," +
                            std::to_string(cb_) + "," + std::to_string(ce_) + "]");
    }
}