/*
 * This file is part of SD-DefSLAM
 * Copyright (C) 2020 Juan J. Gómez Rodríguez, Jose Lamarca Peiro, J. Morlana,
 *                    Juan D. Tardós and J.M.M. Montiel, University of Zaragoza
 *
 * This software is for internal use in the EndoMapper project.
 * Not to be re-distributed.
 */

#ifndef DEFORMABLESLAM_FILTER_H
#define DEFORMABLESLAM_FILTER_H


#include<opencv2/core/core.hpp>

namespace defSLAM {
    class Filter {
        /*
         * This class defines a generic filter to generate a single mask from an input image
         */
    public:
        Filter(){};

        /*
         * Virtual method for generating the mask
         */
        virtual cv::Mat generateMask(const cv::Mat &im) = 0;

        /*
         * Retrieves a short description of the filter
         */
        virtual std::string getDescription() = 0;

    private:
        std::string filterDescription;
    };
}


#endif //DEFORMABLESLAM_FILTER_H
