#include "glog/logging.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "ceres/ceres.h"

#include <System.h>
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight,
                const string &strPathTimes, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps);

void LoadMasks(const string &strPath, vector<string> &vstrMasks);

int main(int argc, char **argv)
{
  if (argc != 6)
  {
    cerr << endl
         << "Usage: ./stereo_Morlana_groundtruth path_to_vocabulary "
            "path_to_settings path_to_left_folder path_to_right_folder "
            "path_to_times_file"
         << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<double> vTimeStamp;

  std::cout << "start load images  " << std::endl;
  LoadImages(string(argv[3]), string(argv[4]), string(argv[5]), vstrImageLeft,
             vstrImageRight, vTimeStamp);
  std::cout << "end load images  " << std::endl;

  if (vstrImageLeft.empty() || vstrImageRight.empty())
  {
    cerr << "ERROR: No images in provided path." << endl;
    return 1;
  }

  if (vstrImageLeft.size() != vstrImageRight.size())
  {
    cerr << "ERROR: Different number of left and right images." << endl;
    return 1;
  }

  // Read rectification parameters
  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    cerr << "ERROR: Wrong path to settings" << endl;
    return -1;
  }

  cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
  fsSettings["LEFT.K"] >> K_l;
  fsSettings["RIGHT.K"] >> K_r;

  fsSettings["LEFT.P"] >> P_l;
  fsSettings["RIGHT.P"] >> P_r;

  fsSettings["LEFT.R"] >> R_l;
  fsSettings["RIGHT.R"] >> R_r;

  fsSettings["LEFT.D"] >> D_l;
  fsSettings["RIGHT.D"] >> D_r;

  int rows_l = fsSettings["LEFT.height"];
  int cols_l = fsSettings["LEFT.width"];
  int rows_r = fsSettings["RIGHT.height"];
  int cols_r = fsSettings["RIGHT.width"];

  if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() ||
      R_r.empty() || D_l.empty() || D_r.empty() || rows_l == 0 || rows_r == 0 ||
      cols_l == 0 || cols_r == 0)
  {
    cerr << "ERROR: Calibration parameters to rectify stereo are missing!"
         << endl;
    return -1;
  }

  cv::Mat M1l, M2l, M1r, M2r;
  cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3),
                              cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
  cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3),
                              cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);

  const size_t nImages = vstrImageLeft.size();

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl
       << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl
       << endl;

  sleep(3);
  // Main loop
  string mask_path = "/home/user/Downloads/Unet11/sequence_organs/resized_pred1/sequence_organs/";
  //string mask_path = "/home/user/Downloads/Unet11/sequence_heart/resized_pred0/sequence_heart/";
  //vector<string> vMaks;
  // LoadMasks(mask_path, vMaks);
  size_t start = 200;
  cv::Mat imLeft, imRight, imLeftRect, imRightRect;
  cv::Mat cnnMask;
  for (size_t ni = start; ni < nImages; ni++)
  {
    std::cout << vstrImageLeft[ni] << " i:  " << ni << std::endl;
    // Read left and right images from file
    imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
    imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);

    cv::Mat imGray;
    cv::cvtColor(imLeft, imGray, cv::COLOR_BGR2GRAY);
    /* cnnMask = cv::imread(vMaks[ni - 200], cv::IMREAD_GRAYSCALE);

    for (int i = 0; i < cnnMask.rows; i++)
    {
      uchar *pMask = cnnMask.ptr<uchar>(i);
      uchar *pImg = imGray.ptr<uchar>(i);
      for (int j = 0; j < cnnMask.cols; j++)
      {
        if (pMask[j] > 125 ||
            pImg[j] > 200)
          pMask[j] = 0;
        else
          pMask[j] = 255;
      }
    }*/

    //cv::erode(cnnMask, cnnMask, getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15)));
    //cv::GaussianBlur(cnnMask, cnnMask, cv::Size(15, 15), 5, 5, cv::BORDER_REFLECT_101);

    //cv::imshow("CNN mask", cnnMask);

    /*cv::Mat imLeftHSV;
    cv::Mat mask(imLeft.rows, imLeft.cols, CV_8UC1);
*/
    auto t1 = std::chrono::high_resolution_clock::now();
    /*   cv::cvtColor(imLeft, imLeftHSV, cv::COLOR_BGR2HSV);

    cv::Mat channels[3], channel2[3];
    cv::split(imLeftHSV, channels);

    for (uint i(0); i < imLeftHSV.rows; i++)
    {
      for (uint j(0); j < imLeftHSV.cols; j++)
      {
        if ((channels[0].at<uchar>(i, j) < 120 &&
             channels[1].at<uchar>(i, j) < 120) ||
            imGray.at<uchar>(i, j) > 225)
        {
          mask.at<uchar>(i, j) = 0;
        }
        else
        {
          mask.at<uchar>(i, j) = 255;
        }
      }
    }*/

    // cv::erode(mask, mask, getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15)));
    // cv::GaussianBlur(mask, mask, cv::Size(15, 15), 5, 5, cv::BORDER_REFLECT_101);

    auto t2 = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

    if (imLeft.empty())
    {
      cerr << endl
           << "Failed to load image at: " << string(vstrImageLeft[ni]) << endl;
      return 1;
    }

    if (imRight.empty())
    {
      cerr << endl
           << "Failed to load image at: " << string(vstrImageRight[ni]) << endl;
      return 1;
    }

    std::chrono::steady_clock::time_point trect1 =
        std::chrono::steady_clock::now();

    cv::Mat _mask;
    cv::remap(imLeft, imLeftRect, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(imRight, imRightRect, M1r, M2r, cv::INTER_LINEAR);
    //cv::remap(cnnMask, cnnMask, M1l, M2l, cv::INTER_LINEAR);

    //imLeftRect = imLeftRect.rowRange(20,imLeft.rows);
    //imRightRect = imRightRect.rowRange(20,imRight.rows);

    std::chrono::steady_clock::time_point trect2 =
        std::chrono::steady_clock::now();

    double tframe = vTimeStamp[ni];

    if (imLeftRect.channels() == 1)
    {
      cv::cvtColor(imLeftRect, imLeftRect, cv::COLOR_GRAY2RGB);
      cv::cvtColor(imRightRect, imRightRect, cv::COLOR_GRAY2RGB);
    }

    cv::Mat __mask(imLeftRect.rows, imLeftRect.cols, CV_8UC1, cv::Scalar(0));
    cv::Rect roi(30, 20, imLeftRect.cols - 60, imLeftRect.rows - 40);
    __mask(roi) = cv::Scalar(255);
    cv::remap(__mask, _mask, M1l, M2l, cv::INTER_LINEAR);

    //cv::remap(__mask,__mask, M1l, M2l, cv::INTER_LINEAR);
    SLAM.TrackMonocularGT(imLeftRect, imRightRect, ni, _mask);

    // Wait to load the next frame
    double T = 0;
    if (ni < nImages - 1)
      T = vTimeStamp[ni + 1] - tframe;
    else if (ni > 0)
      T = tframe - vTimeStamp[ni - 1];

    //  if(ttrack<T)
    //      usleep((T-ttrack)*1e6);
  }

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages; ni++)
  {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl
       << endl;
  cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  cout << "mean tracking time: " << totaltime / nImages << endl;

  // Save camera trajectory
  //SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

  return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight,
                const string &strPathTimes, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
  ifstream fTimes;

  fTimes.open(strPathTimes);
  vTimeStamps.reserve(5000);
  vstrImageLeft.reserve(5000);
  vstrImageRight.reserve(5000);
  std::cout << strPathTimes << std::endl;
  while (!fTimes.eof())
  {
    string s;
    getline(fTimes, s);
    if (!s.empty())
    {
      stringstream ss;
      ss << s;
      string name = ss.str();
      for (uint i(0); i < 6; i++)
        name.pop_back();
      vstrImageLeft.push_back(strPathLeft + "/stereo_im_l_" + name + ".png");
      vstrImageRight.push_back(strPathRight + "/stereo_im_r_" + name + ".png");

      //  std::cout << name<< std::endl;

      double t;
      ss >> t;
      vTimeStamps.push_back(t / 1e6);
    }
  }
}

void LoadMasks(const string &strPath, vector<string> &vstrMasks)
{
  //vstrMasks.resize(1000);
  for (int i = 0; i < 1000; i++)
  {
    if (i < 800)
      vstrMasks.push_back(strPath + "stereo_im_l_000" + to_string(i + 200) + ".png");
    else
      vstrMasks.push_back(strPath + "stereo_im_l_00" + to_string(i + 200) + ".png");
  }
}
