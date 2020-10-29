/*
 * This file is part of SD-DefSLAM
 * Copyright (C) 2020 Juan J. Gómez Rodríguez, Jose Lamarca Peiro, J. Morlana,
 *                    Juan D. Tardós and J.M.M. Montiel, University of Zaragoza
 *
 * This software is for internal use in the EndoMapper project.
 * Not to be re-distributed.
 */

#include <opencv2/opencv.hpp>

#include "BrightMask.h"

#include <iostream>


namespace defSLAM {

    cv::Mat BrightMask::generateMask(const cv::Mat &im) {
        cv::Mat imGray = im, mask;
        if(imGray.channels()==3){
            cvtColor(imGray,imGray,cv::COLOR_BGR2GRAY);
        }
        else if(imGray.channels()==4){
            cvtColor(imGray,imGray,cv::COLOR_BGR2GRAY);
        }

        cv::threshold(imGray,mask,th_,255,cv::THRESH_BINARY_INV);

        cv::erode(mask, mask, getStructuringElement(cv::MORPH_RECT, cv::Size(21, 21)));
        cv::GaussianBlur(mask, mask, cv::Size(11, 11), 5, 5, cv::BORDER_REFLECT_101);

        return mask;
    }

    std::string BrightMask::getDescription() {
        return std::string("Bright mask with th_ = " + std::to_string(th_));
    }
}

