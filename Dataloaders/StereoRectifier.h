#pragma once

#include <string>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

#include <fstream>

using namespace std;
using namespace cv;

namespace dataset
{
    using std::string;
    class StereoRectifier
    {
        /*********
         * Class which uses opencv2 to rectify stereo pairs
         * Author: Jose Lamarca (jlamarca@unizar.es)
         * *******/
    public:
        StereoRectifier()
            : checkEpipoles_(false)
        {
        }

        void setExtrinsicsFromFile(string fileName)
        {
            extrinsicCal_ = ReadMatFromTxt(fileName, 4, 3);
            R = extrinsicCal_(cv::Range(0, 3), cv::Range(0, 3));
            t = extrinsicCal_.row(3).t();
            //R = extrinsicCal_(cv::Range(0, 3), cv::Range(0, 3)).t();
            //t = -R * extrinsicCal_.row(3).t();
            std::cout << R << std::endl;
            std::cout << std::endl;
            std::cout << t << std::endl;
            std::cout << std::endl;
        };

        void setLeftIntrinsicsFromFile(string fileName)
        {
            leftCal_ = ReadMatFromTxt(fileName, 3, 3);
            leftDistorsion_ = getlineMatFromTxt(fileName, 4, 3);
        };

        void setRightIntrinsicsFromFile(string fileName)
        {
            rightCal_ = ReadMatFromTxt(fileName, 3, 3);
            rightDistorsion_ = getlineMatFromTxt(fileName, 4, 3);
        };

        void checkEpipoles()
        {
            checkEpipoles_ = true;
        };
        /******
         * It receives the left and right images and rectify them giving them back in leftImRect and rightImRect
         * ******/
        void rectify(cv::Mat &leftIm, cv::Mat &rightIm, cv::Mat &leftImRect, cv::Mat &rightImRect)
        {
            // This is done just once
            if (R_l.empty())
            {
                cv::stereoRectify(leftCal_, leftDistorsion_, rightCal_, rightDistorsion_, leftIm.size(), R, t, R_l, R_r, P_l, P_r, Q, cv::CALIB_ZERO_DISPARITY, -1, leftIm.size(), 0, 0);
                cv::initUndistortRectifyMap(leftCal_, leftDistorsion_, R_l, P_l.rowRange(0, 3).colRange(0, 3),
                                            leftIm.size(), CV_32F, M1l, M2l);
                cv::initUndistortRectifyMap(rightCal_, rightDistorsion_, R_r, P_r.rowRange(0, 3).colRange(0, 3),
                                            rightIm.size(), CV_32F, M1r, M2r);
            }
            cv::remap(leftIm, leftImRect, M1l, M2l, cv::INTER_LINEAR);
            cv::remap(rightIm, rightImRect, M1r, M2r, cv::INTER_LINEAR);
            if (checkEpipoles_)
            {
                drawPatternlines(leftImRect);
                drawPatternlines(rightImRect);
            }
        };

    private:
        /*****
     * Function to read Matrix in cv::Mat format
     * taken from:
     * https://answers.opencv.org/question/42770/how-to-read-matrix-from-text-file-to-mat-in-opencv/
     * ***/
        Mat ReadMatFromTxt(string filename, int rows, int cols)
        {
            double m;
            Mat out = Mat::zeros(rows, cols, CV_64FC1); //Matrix to store values

            ifstream fileStream(filename);
            int cnt = 0; //index starts from 0
            while (fileStream >> m)
            {
                int temprow = cnt / cols;
                int tempcol = cnt % cols;
                out.at<double>(temprow, tempcol) = m;
                cnt++;
            }
            return out;
        }

        /*****
        * Function to read Matrix in cv::Mat format from one single line of the document
        * ***/
        Mat getlineMatFromTxt(string filename, int cols, int lineNumber)
        {
            Mat out = Mat::zeros(1, cols, CV_64FC1); //Matrix to store values

            ifstream fileStream(filename);
            int cnt = 0; //index starts from 0
            if (fileStream.is_open())
            {
                string line;
                while (getline(fileStream, line))
                {
                    if (cnt == lineNumber)
                    {
                        stringstream ss(line);
                        for (int i = 0; i < 4; i++)
                        {
                            ss >> out.at<double>(0, i);
                        }
                    }
                    cnt++;
                }

                return out;
            }
        }
        /******
         * function that draws lines in an image
         * ****/
        void drawPatternlines(cv::Mat &image) const
        {
            for (int i(5); i < image.rows; i = i + 20)
            {
                cv::line(image, cv::Point(0, i), cv::Point(image.cols - 1, i), cv::Scalar(180, 10, 10));
            }
        }

    private:
        cv::Mat extrinsicCal_;         // extrinsic calibration
        cv::Mat leftCal_;              // left Calibration
        cv::Mat rightCal_;             // right Calibration
        cv::Mat leftDistorsion_;       // left Distorsion
        cv::Mat rightDistorsion_;      // right Distorsion
        cv::Mat R;                     // Rotation between the first and the second camera
        cv::Mat t;                     // Translation between the first and the second camera
        cv::Mat R_l, R_r, P_l, P_r, Q; // Rectification parameters
        cv::Mat M1l, M2l, M1r, M2r;    // Map Parameters
        bool checkEpipoles_;
    };
} // namespace dataset