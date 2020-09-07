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
        ~Filter(){};

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
