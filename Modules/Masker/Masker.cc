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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#include <istream>

#include "Masker.h"
#include "BorderMask.h"
#include "BrightMask.h"
#include "CnnSegmentation.h"

namespace defSLAM
{

    void Masker::loadFromTxt(std::string path)
    {
        std::cout << std::endl
                  << "Loading filters: " << path << std::endl;
        //Open filter file
        std::ifstream filterFile(path);

        if (filterFile.is_open())
        {
            std::string line;
            while (getline(filterFile, line))
            {
                std::istringstream ss(line);

                //Get filter name
                std::string name;
                ss >> name;

                if (name == "CNN")
                {
                    CnnSegmentation *cnn = new CnnSegmentation();
                    ss >> name;
                    cnn->loadModel(name);
                    std::unique_ptr<Filter> f(cnn);

                    addFilter(f);
                }
                else if (name == "BorderFilter")
                {
                    std::string rb, re, cb, ce, th;
                    ss >> rb >> re >> cb >> ce >> th;
                    std::unique_ptr<Filter> f(new BorderMask(stoi(rb), stoi(re), stoi(cb), stoi(ce), stoi(th)));
                    addFilter(f);
                }
                else if (name == "BrightFilter")
                {
                    std::string thLo;
                    ss >> thLo;
                    std::unique_ptr<Filter> f(new BrightMask(stoi(thLo)));
                    addFilter(f);
                }
            }

            filterFile.close();
        }
    }

    void Masker::addFilter(std::unique_ptr<Filter> &f)
    {
        filters_.push_back(std::move(f));
    }

    void Masker::deleteFilter(size_t idx)
    {
        filters_.erase(filters_.begin() + idx);
    }

    cv::Mat Masker::mask(const cv::Mat &im)
    {
        //Generates an empty mask (all values set to 0)
        cv::Mat mask(im.rows, im.cols, CV_8U, cv::Scalar(255));

        //Apply each filter
        for (auto &f : filters_)
        {
            cv::bitwise_and(mask, f->generateMask(im), mask);
        }

        return mask;
    }

    std::string Masker::printFilters()
    {
        std::string msg("List of filters (" + std::to_string(filters_.size()) + "):\n");

        for (auto &f : filters_)
        {
            msg += "\t-" + f->getDescription() + "\n";
        }

        return msg;
    }
} // namespace defSLAM