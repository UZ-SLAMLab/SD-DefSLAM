/*
 * This file is part of SD-DefSLAM
 * Copyright (C) 2020 Juan J. Gómez Rodríguez, Jose Lamarca Peiro, J. Morlana,
 *                    Juan D. Tardós and J.M.M. Montiel, University of Zaragoza
 *
 * This software is for internal use in the EndoMapper project.
 * Not to be re-distributed.
 */

#ifndef DEFORMABLESLAM_BORDERMASK_H
#define DEFORMABLESLAM_BORDERMASK_H

#include<opencv2/core/core.hpp>

#include "Filter.h"

namespace defSLAM {
    class BorderMask : public Filter{
        /*
         * This class defines a mask over the outer borders of the image. It masks out:
         *  -rb_ and re_ rows from the beginning and the end respectively
         *  -cb_ and ce_ rows from the beginning and the end respectively
         */

    public:
        BorderMask(int rb, int re, int cb, int ce, int th) :
                   rb_(rb),re_(re),cb_(cb),ce_(ce), th_(th) {}

        cv::Mat generateMask(const cv::Mat &im);

        std::string getDescription();

    private:
        int th_;
        int rb_,re_,cb_,ce_;
    };
}


#endif //DEFORMABLESLAM_BORDERMASK_H
