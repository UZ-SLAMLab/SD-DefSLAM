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

#include "BrightMask.h"

#include <iostream>


namespace defSLAM {

    cv::Mat BrightMask::generateMask(const cv::Mat &im) {
        cv::Mat imGray = im;
        if(imGray.channels()==3){
            cvtColor(imGray,imGray,cv::COLOR_BGR2GRAY);
        }
        else if(imGray.channels()==4){
            cvtColor(imGray,imGray,cv::COLOR_BGR2GRAY);
        }

        cv::Mat mask;
        cv::threshold(imGray,mask,th_,255,cv::THRESH_BINARY_INV);

        return mask;
    }

    std::string BrightMask::getDescription() {
        return std::string("Bright mask with th_ = " + std::to_string(th_));
    }
}

