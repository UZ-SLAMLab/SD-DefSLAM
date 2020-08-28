/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

// Demo program showing how libelas can be used, try "./elas -h" for help

#include <iostream>
#include "elas.h"
#include "image.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "dataset/dataloader.h"
#include "string"
#include "boost/filesystem.hpp"
using namespace std;
using std::string;

int main(int argc, char **argv)
{
    /*****
     * Usage:
     * ./elasHamlyn <folder_images_Left> <folder_images_Right> <pattern_images_Left> <pattern_images_Right>
     * *****/

    dataset::DataLoader *hamlynloader;
    string folderOutput;
    auto drawPattern(false); // Activate to generate results with horizontal lines (epipolar lines)

    if (argc == 9)
    {
        string folderImagesLeft(argv[1]);
        string folderImagesRight(argv[2]);
        string patternImagesLeft(argv[3]);
        string patternImagesRight(argv[4]);
        string leftCalibration(argv[5]);
        string rightCalibration(argv[6]);
        string extrinsicCalibration(argv[7]);
        hamlynloader = new dataset::DataLoader(folderImagesLeft, folderImagesRight, patternImagesLeft,
            patternImagesRight, leftCalibration, rightCalibration,
            extrinsicCalibration, drawPattern);

        folderOutput = string(argv[8]);
    }
    else if (argc == 6)
    {
        string videoPair(argv[1]);
        string leftCalibration(argv[2]);
        string rightCalibration(argv[3]);
        string extrinsicCalibration(argv[4]);
        hamlynloader = new dataset::DataLoader(videoPair, leftCalibration, rightCalibration,
            extrinsicCalibration, drawPattern);
        folderOutput = string(argv[5]);
    }
    else if (argc == 7)
    {
        string videoLeft(argv[1]);
        string videoRight(argv[2]);

        string leftCalibration(argv[3]);
        string rightCalibration(argv[4]);
        string extrinsicCalibration(argv[5]);
        hamlynloader = new dataset::DataLoader(videoLeft, videoRight, leftCalibration, rightCalibration,
            extrinsicCalibration, drawPattern);
        folderOutput = string(argv[6]);
    }
    else
    {
        std::cout << "args passed " << argc << std::endl;
        cout << endl;
        cout << "ELAS demo program usage: " << endl;
        cout << "./elasHamlyn " << endl;
        cout << endl;
        cout << endl;
    }

    std::cout << hamlynloader->size() << std::endl;
    boost::filesystem::path dstFolder(folderOutput);
    boost::filesystem::create_directory(dstFolder);

    for (size_t i(0); i < hamlynloader->size(); i++)
    {
        auto stereoPair = hamlynloader->operator[](i);

        cv::Mat grayLeft, grayRight;
        if (stereoPair.first.channels() > 1)
        {
            cv::cvtColor(stereoPair.first, grayLeft, COLOR_BGR2GRAY);
            cv::cvtColor(stereoPair.second, grayRight, COLOR_BGR2GRAY);
        }
        else
        {
            grayLeft = stereoPair.first.clone();
            grayRight = stereoPair.second.clone();
        }

        cv::imshow("left image", stereoPair.first);
        cv::imshow("right image", stereoPair.second);
        std::ostringstream out;
        out << std::internal << std::setfill('0') << std::setw(6)
            << uint(i);
        cv::imwrite(folderOutput + "/left_rect_image_" + out.str() + ".png", stereoPair.first);
        cv::imwrite(folderOutput + "/right_rect_image_" + out.str() + ".png", stereoPair.second);

        cv::waitKey(10);
    }
    delete hamlynloader;
    return 0;
}
