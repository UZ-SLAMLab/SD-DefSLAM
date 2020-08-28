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

using namespace std;
using std::string;

// compute disparities of pgm image input pair file_1, file_2
cv::Mat process(cv::Mat &imLeft, cv::Mat &imRight)
{
    // load images
    image<uchar> *I1, *I2;
    I1 = new image<uchar>(imLeft.cols, imLeft.rows);
    I1->data = imLeft.data;
    I2 = new image<uchar>(imRight.cols, imRight.rows);
    I2->data = imRight.data;
    // check for correct size
    if (I1->width() <= 0 || I1->height() <= 0 || I2->width() <= 0 || I2->height() <= 0 ||
        I1->width() != I2->width() || I1->height() != I2->height())
    {
        cout << "ERROR: Images must be of same size, but" << endl;
        cout << "       I1: " << I1->width() << " x " << I1->height() << ", I2: " << I2->width() << " x " << I2->height() << endl;
        return cv::Mat();
    }

    // get image width and height
    int32_t width = I1->width();
    int32_t height = I1->height();

    // allocate memory for disparity images
    const int32_t dims[3] ={ width, height, width }; // bytes per line = width
    float *D1_data = (float *)malloc(width * height * sizeof(float));
    float *D2_data = (float *)malloc(width * height * sizeof(float));

    // process
    Elas::parameters param;
    param.postprocess_only_left = true;
    param.disp_max = width / 2;
    Elas elas(param);
    elas.process(I1->data, I2->data, D1_data, D2_data, dims);

    // find maximum disparity for scaling output disparity images to [0..255]
    float disp_max = 15;
    param.disp_min;
    for (int32_t i = 0; i < width * height; i++)
    {
        if (D1_data[i] > disp_max)
            disp_max = D1_data[i];
        /*if (D2_data[i] > disp_max)
          disp_max = D2_data[i];*/
    }

    // copy float to uchar
    image<uchar> *D1 = new image<uchar>(width, height);
    image<uchar> *D2 = new image<uchar>(width, height);

    for (int32_t i = 0; i < width * height; i++)
    {
        D1->data[i] = (uint8_t)max(255.0 * D1_data[i] / disp_max, 0.0);
        D2->data[i] = (uint8_t)max(255.0 * D2_data[i] / disp_max, 0.0);
    }

    cv::Mat img_color;
    cv::Mat img_in(height, width, CV_8UC1, D1->data, 0);
    cv::imshow("MyDisp", img_in);
    cv::Mat img_norm;
    cv::normalize(img_in, img_norm, 0,255, NORM_MINMAX, CV_8UC1);

    applyColorMap(img_norm, img_color, COLORMAP_JET);
    cv::imshow("DispInColor", img_color);

    // free memory
    I1 = nullptr;
    I2 = nullptr;
    delete D1;
    delete D2;
    free(D1_data);
    free(D2_data);
    return img_in;
}

int main(int argc, char **argv)
{
    /*****
     * Usage:
     * ./elasHamlyn <folder_images_Left> <folder_images_Right> <pattern_images_Left> <pattern_images_Right>
     * *****/

    dataset::DataLoader *hamlynloader;
    auto drawPattern(false); // Activate to generate results with horizontal lines (epipolar lines)
    if (argc == 8)
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
    }
    else if (argc == 5)
    {
        string videoPair(argv[1]);
        string leftCalibration(argv[2]);
        string rightCalibration(argv[3]);
        string extrinsicCalibration(argv[4]);
        hamlynloader = new dataset::DataLoader(videoPair, leftCalibration, rightCalibration,
            extrinsicCalibration, drawPattern);
    }
    else if (argc == 6)
    {
        string videoLeft(argv[1]);
        string videoRight(argv[2]);

        string leftCalibration(argv[3]);
        string rightCalibration(argv[4]);
        string extrinsicCalibration(argv[5]);
        hamlynloader = new dataset::DataLoader(videoLeft, videoRight, leftCalibration, rightCalibration,
            extrinsicCalibration, drawPattern);
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

        clock_t start = clock();
        process(grayLeft,grayRight);
        clock_t end = clock();

        double diffticks = end - start;
        double diffms = diffticks / (CLOCKS_PER_SEC / 1000);
        cv::imshow("left image", stereoPair.first);
        cv::imshow("right image", stereoPair.second);

        cv::waitKey(0);
        std::cout << "time " << diffms << std::endl;
    }
    delete hamlynloader;
    return 0;
}
