// This software is for internal use in the EndoMapper project.
// Not to be re-distributed.

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
