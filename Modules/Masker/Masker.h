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


#ifndef DEFORMABLESLAM_MASKER_H
#define DEFORMABLESLAM_MASKER_H

#include<opencv2/core/core.hpp>

#include "Filter.h"

namespace defSLAM {
    class Masker {
        /*
         * This class colects a set of filter to generate a global mask from all of them
         */
    public:
        Masker(){};
        ~Masker();

        /*
         * Load filters from a .txt file with the following format:
         * <filterName> <param_1> <param_2>
         */
        void loadFromTxt(std::string path);

        /*
         * Adds a filter to the masker
         */
        void addFilter(Filter* f);

        /*
         * Removes the filter at pos idx
         */
        void deleteFilter(size_t idx);

        /*
         * Applies all filters stored and generates a global mask
         */
        cv::Mat mask(const cv::Mat& im);

        std::string printFilters();

    private:
        std::vector<Filter*> filters_;
    };
}

#endif //DEFORMABLESLAM_MASKER_H
