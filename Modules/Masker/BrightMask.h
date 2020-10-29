/*
 * This file is part of SD-DefSLAM
 * Copyright (C) 2020 Juan J. Gómez Rodríguez, Jose Lamarca Peiro, J. Morlana,
 *                    Juan D. Tardós and J.M.M. Montiel, University of Zaragoza
 *
 * This software is for internal use in the EndoMapper project.
 * Not to be re-distributed.
 */

#ifndef DEFORMABLESLAM_BRIGHTMASK_H
#define DEFORMABLESLAM_BRIGHTMASK_H

#include<opencv2/core/core.hpp>

#include "Filter.h"

namespace defSLAM {

    class BrightMask : public Filter{
        /*
         * This class defines a mask over the brightest pixel on an image. Pixels masked out must
         * have a value grater than th_
         */
    public:
        BrightMask(int th) : th_(th) {}

        cv::Mat generateMask(const cv::Mat& im);

        std::string getDescription();
    private:
        int th_;
    };
}


#endif //DEFORMABLESLAM_BRIGHTMASK_H
