#pragma once
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include <StereoRectifier.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
namespace dataset
{
    namespace Hamlyn
    {
        using std::string;
        using std::vector;
        using namespace boost::filesystem;

        class DataLoader
        {
            /*********
         * Class which loads a dataset with stereo pairs withour rectifications 
         * and rectify them. Tested in Hamlyn dataset.
         * Author: Jose Lamarca (jlamarca@unizar.es)
         * *******/
            typedef vector<path> vec;

        public:
            DataLoader() = default;

            /********
         * videoName: Video with both left and right images.
         * leftCalibration: Text file that contains both the intrinsic parameters and the distorsion of left camera
         * rightCalibration: Text file that contains both the intrinsic parameters and the distorsion of left camera
         * extrinsicCalibration: Text file that contains the relative transformation between camera 1 and 2
         * checkEpipoles: Draw horizontal lines in the images.
         * *****/
            DataLoader(const string &videoName,
                       const string &leftCalibration, const string &rightCalibration,
                       const string &extrinsicCalibration, bool checkEpipoles = true)
            {
                oneVideo_ = true;
                videoName_ = videoName;
                stereoRectifier_.setExtrinsicsFromFile(extrinsicCalibration);
                stereoRectifier_.setLeftIntrinsicsFromFile(leftCalibration);
                stereoRectifier_.setRightIntrinsicsFromFile(rightCalibration);
                if (checkEpipoles)
                    stereoRectifier_.checkEpipoles();
                preprocessImages();
            }

            /********
         * videoLeft: Video with left images.
         * videoRight: Video with right images.
         * leftCalibration: Text file that contains both the intrinsic parameters and the distorsion of left camera
         * rightCalibration: Text file that contains both the intrinsic parameters and the distorsion of left camera
         * extrinsicCalibration: Text file that contains the relative transformation between camera 1 and 2
         * checkEpipoles: Draw horizontal lines in the images.
         * *****/
            DataLoader(const string &videoLeft,
                       const string &videoRight,
                       const string &leftCalibration, const string &rightCalibration,
                       const string &extrinsicCalibration, bool checkEpipoles = true)
            {
                oneVideo_ = false;
                videoName_ = videoLeft;
                videoRight_ = videoRight;

                stereoRectifier_.setExtrinsicsFromFile(extrinsicCalibration);
                stereoRectifier_.setLeftIntrinsicsFromFile(leftCalibration);
                stereoRectifier_.setRightIntrinsicsFromFile(rightCalibration);
                if (checkEpipoles)
                    stereoRectifier_.checkEpipoles();
                preprocessImages();
            }

            /********
         * folderLeft: folder with left images
         * folderRight: folder with right images
         * namePatternLeft: pattern of the name for left images
         * namePatternRight: pattern of the name for right images
         * leftCalibration: Text file that contains both the intrinsic parameters and the distorsion of left camera
         * rightCalibration: Text file that contains both the intrinsic parameters and the distorsion of left camera
         * extrinsicCalibration: Text file that contains the relative transformation between camera 1 and 2
         * checkEpipoles: Draw horizontal lines in the images.
         * *****/
            DataLoader(const string &folderLeft, const string &folderRight,
                       const string &namePatternLeft, const string &namePatternRight,
                       const string &leftCalibration, const string &rightCalibration,
                       const string &extrinsicCalibration, bool checkEpipoles = true)
            {
                path folderLeftPath(folderLeft);
                path folderRightPath(folderRight);
                assert(!exists(folderLeftPath));
                assert(!exists(folderRightPath));
                vec vLeft; // so we can sort them later
                copy(directory_iterator(folderLeftPath), directory_iterator(), back_inserter(vLeft));
                vec vRight; // so we can sort them later
                copy(directory_iterator(folderRightPath), directory_iterator(), back_inserter(vRight));
                sort(vLeft.begin(), vLeft.end());   // sort, since directory iteration
                sort(vRight.begin(), vRight.end()); // sort, since directory iteration

                for (size_t i(0); i < vLeft.size(); i++)
                {
                    if (vLeft[i].string().find(namePatternLeft) != std::string::npos)
                    {
                        imagesLeft_.push_back(std::move(vLeft[i]));
                    }
                    if (vRight[i].string().find(namePatternRight) != std::string::npos)
                    {
                        imagesRight_.push_back(std::move(vRight[i]));
                    }
                }
                assert(imagesLeft_.size() != imagesRight_.size()); // Check that both list of images have same size
                stereoRectifier_.setExtrinsicsFromFile(extrinsicCalibration);
                stereoRectifier_.setLeftIntrinsicsFromFile(leftCalibration);
                stereoRectifier_.setRightIntrinsicsFromFile(rightCalibration);
                if (checkEpipoles)
                    stereoRectifier_.checkEpipoles();
                preprocessImages();
            }

            ~DataLoader()
            {
                delete[] imageLeftRect;
                delete[] imageRightRect;
            }
            /*********
         * Return the number of elements in the vector of names imagesLeft
         * ******/
            size_t size() const
            {
                return frames_;
            }

            std::pair<cv::Mat, cv::Mat> operator[](uint i)
            {
                return std::make_pair(imageLeftRect[i], imageRightRect[i]);
            }

        private:
            void preprocessImages()
            {
                if (imagesLeft_.size() > 0)
                {
                    imageLeftRect = new cv::Mat[imagesLeft_.size()];
                    imageRightRect = new cv::Mat[imagesLeft_.size()];

                    for (size_t i(0); i < imagesLeft_.size(); i++)
                    {
                        cv::Mat I1i = cv::imread(imagesLeft_[i].string(), cv::IMREAD_GRAYSCALE);
                        cv::Mat I2i = cv::imread(imagesRight_[i].string(), cv::IMREAD_GRAYSCALE);
                        stereoRectifier_.rectify(I1i, I2i, imageLeftRect[i], imageRightRect[i]);
                        cv::resize(imageRightRect[i], imageRightRect[i], imageLeftRect[i].size());
                    }
                    frames_ = imagesLeft_.size();
                }
                else
                {
                    cv::VideoCapture video(videoName_);
                    uint frames(0);
                    while (video.isOpened())
                    {
                        cv::Mat imTogether;
                        video >> imTogether;
                        if (imTogether.empty())
                            break;
                        frames++;
                    }
                    imageLeftRect = new cv::Mat[frames];
                    imageRightRect = new cv::Mat[frames];

                    video.open(videoName_);
                    cv::VideoCapture videoRight;
                    if (!oneVideo_)
                    {
                        videoRight.open(videoRight_);
                    }
                    frames = 0;
                    while (video.isOpened())
                    {
                        cv::Mat imTogether;
                        video >> imTogether;
                        if (imTogether.empty())
                            break;
                        cv::Mat I1i;
                        cv::Mat I2i;
                        if (oneVideo_)
                        {
                            int rows = imTogether.rows;
                            int cols = imTogether.cols;
                            cv::Rect leftROI(0, 0, cols / 2, rows);
                            cv::Rect rightROI(cols / 2, 0, cols / 2, rows);
                            I1i = imTogether(leftROI);
                            I2i = imTogether(rightROI);
                        }
                        else
                        {
                            I1i = imTogether;
                            videoRight >> I2i;
                        }

                        stereoRectifier_.rectify(I1i, I2i, imageLeftRect[frames], imageRightRect[frames]);
                        frames++;
                    }
                    frames_ = frames;
                }
            }

        private:
            vec imagesLeft_;  // Name of the left images
            vec imagesRight_; // Name of the right images

            string videoName_;
            string videoRight_;

            StereoRectifier stereoRectifier_; // Rectifier of stereo pairs

            cv::Mat *imageLeftRect;  // Left images
            cv::Mat *imageRightRect; // Right images

            int frames_;
            bool oneVideo_;
        }; // namespace dataset
    }      // namespace Hamlyn
} // namespace dataset