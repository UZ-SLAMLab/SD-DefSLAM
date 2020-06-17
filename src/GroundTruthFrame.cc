/**
* This file is part of DefSLAM.
*
* Copyright (C) 2017-2020 Jose Lamarca Peiro <jlamarca at unizar dot es> (University
*of Zaragoza)
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

#include "GroundTruthFrame.h"
#include "GroundTruthCalculator.h"
#include "MinMedianFilter.h"
#include "SmootherMLS.h"
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "CC_MAC.h"
#include "DeformationMapPoint.h"
#include "MapPoint.h"

#include "opencv2/calib3d.hpp"
#include "opencv2/core/core.hpp"

namespace defSLAM
{

  using ORB_SLAM2::MapPoint;

  GroundTruthFrame::GroundTruthFrame(const cv::Mat &imGray,
                                     const double &timeStamp,
                                     ORBextractor *extractor, ORBVocabulary *voc,
                                     cv::Mat &K, cv::Mat &distCoef,
                                     const float &bf, const float &thDepth,
                                     const cv::Mat &ImRGB, const cv::Mat &ImRight,
                                     cv::Mat _mask, int action)
      : Frame(imGray, timeStamp, extractor, voc, K, distCoef, bf, thDepth, ImRGB,
              _mask, action),
        tempx_(TEMPX), tempy_(TEMPY), margin_(MARGIN), searchx_(SEARCHX),
        searchy_(tempy_ + margin_), threshold_(THRESHOLD)
  {
    this->imRight = (ImRight);
    // Do not use stereo to initialize
    this->StereoAvailable = (false);
    isDepth = false;
  }

  GroundTruthFrame::GroundTruthFrame(
      const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
      ORB_SLAM2::ORBextractor *extractorLeft,
      ORB_SLAM2::ORBextractor *extractorRight, ORB_SLAM2::ORBVocabulary *voc,
      cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,
      const cv::Mat &ImRGB, cv::Mat _mask)
      : Frame(imLeft, imRight, timeStamp, extractorLeft, extractorRight, voc, K,
              distCoef, bf, thDepth, ImRGB, _mask),
        tempx_(TEMPX), tempy_(TEMPY), margin_(MARGIN), searchx_(SEARCHX),
        searchy_(tempy_ + margin_), threshold_(THRESHOLD)
  {
    this->imRight = (imRight);
    // Use stereo to initialize
    this->StereoAvailable = (true);
    isDepth = false;
  }

  GroundTruthFrame::GroundTruthFrame(const GroundTruthFrame &frame)
      : Frame(frame), tempx_(frame.tempx_), tempy_(frame.tempy_),
        margin_(frame.margin_), searchx_(frame.searchx_),
        searchy_(tempy_ + margin_), threshold_(frame.threshold_)
  {
    isDepth = false;
  }

  GroundTruthFrame::GroundTruthFrame(const cv::Mat &imGray,
                                     const double &timeStamp,
                                     ORBextractor *extractor, ORBVocabulary *voc,
                                     cv::Mat &K, cv::Mat &distCoef,
                                     const float &bf, const float &thDepth,
                                     const cv::Mat &ImRGB, const cv::Mat &imDepth,
                                     bool isDepth, cv::Mat _mask)
      : Frame(imGray, timeStamp, extractor, voc, K, distCoef, bf, thDepth, ImRGB,
              _mask),
        tempx_(TEMPX), tempy_(TEMPY), margin_(MARGIN), searchx_(SEARCHX),
        searchy_(tempy_ + margin_), threshold_(THRESHOLD), isDepth(isDepth),
        imDepth(imDepth.clone())
  {

    // Use stereo to initialize
    this->StereoAvailable = (false);
  }

  double GroundTruthFrame::Estimate3DScale(Map *map)
  {
    const vector<MapPoint *> &vpMPs = map->GetAllMapPoints();

    set<MapPoint *> spRefMPs(vpMPs.begin(), vpMPs.end());

    posMono_.clear();
    posStereo_.clear();

    cv::Mat left_disp;
    std::vector<std::vector<float>> posMonoInit_;
    std::vector<std::vector<float>> posStereoInit_;
    posMono_.reserve(posMonoInit_.size());
    posStereo_.reserve(posStereoInit_.size());
    std::vector<float> zs;
    for (vector<MapPoint *>::const_iterator sit = vpMPs.begin(),
                                            send = vpMPs.end();
         sit != send; sit++)
    {

      if (!static_cast<DeformationMapPoint *>(*sit)->getFacet())
      {
        continue;
      }

      if ((*sit)->isBad())
        continue;
      // OpenCV_Template Matching tutorial :
      // https://docs.opencv.org/master/de/da9/tutorial_template_matching.html
      cv::Mat pos = (*sit)->GetWorldPos();
      cv::KeyPoint kp = this->ProjectPoints(pos);
      if (kp.pt.x < 0)
        continue;
      if (isDepth)
      {
        float dpth = imDepth.at<float>(kp.pt.y, kp.pt.x);
        const cv::Mat Pc = mRcw * pos + mtcw;
        std::vector<float> pm;
        pm.push_back(Pc.at<float>(0));
        pm.push_back(Pc.at<float>(1));
        pm.push_back(Pc.at<float>(2));
        std::vector<float> ps;
        ps.push_back(dpth * (((float)kp.pt.x - cx) / fx));
        ps.push_back(dpth * (((float)kp.pt.y - cy) / fy));
        ps.push_back(dpth);
        std::unique_lock<std::mutex> lck(mutexPoints);
        posMonoInit_.push_back(pm);
        posStereoInit_.push_back(ps);
        continue;
      }
      if (kp.pt.x < 0)
        continue;
      if (kp.pt.y < 0)
        continue;
      if (kp.pt.y > this->imRight.rows - 0)
        continue;
      if (kp.pt.x > this->imRight.cols - 60)
        continue;
      int tempx(tempx_), tempy(tempy_);
      int searchx(searchx_), searchy(searchy_);

      if (((kp.pt.x - tempx / 2) < 20) or ((kp.pt.y - tempy / 2) < 0) or
          ((kp.pt.x + tempx / 2) > this->imRight.cols) or
          ((kp.pt.y + tempy / 2) > this->imRight.rows))
        continue;

      cv::Rect cropRect(kp.pt.x - tempx / 2, kp.pt.y - tempy / 2, tempx, tempy);
      int finx = (kp.pt.x - searchx);
      int finy = (kp.pt.y - searchy);
      int finxright = kp.pt.x + mbf / 4;
      int finyright = finy + searchy * 2;
      if (finx < 0)
        finx = 0;
      if (finy < 0)
        finy = 0;
      if (finxright > this->imRight.cols)
        searchx = float(this->imRight.cols - 1 - finx);
      if (finyright > this->imRight.rows)
        searchy = float(this->imRight.rows - 1 - finy) / 2;

      cv::Rect SearchRegion(finx, finy, searchx, searchy * 2);
      cv::Mat tmp = ImGray(cropRect);
      cv::Mat SearchImage = this->imRight(SearchRegion);
      cv::Mat result;
      cv::matchTemplate(SearchImage, tmp, result, cv::TM_CCORR_NORMED);
      cv::Point matchLoc;

      double minVal;
      double maxVal;
      cv::Point minLoc;
      cv::Point maxLoc;
      cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

      double CorrelationThreshold(threshold_);

      if ((maxVal < CorrelationThreshold))
        continue;

      matchLoc = maxLoc;

      matchLoc.x = matchLoc.x + finx + tempx / 2;
      matchLoc.y = matchLoc.y + finy + tempy / 2;

      float disp(std::abs(matchLoc.x - kp.pt.x));

      const cv::Mat Pc = mRcw * pos + mtcw;
      std::vector<float> pm;
      pm.reserve(3);
      pm.push_back(Pc.at<float>(0));
      pm.push_back(Pc.at<float>(1));
      pm.push_back(Pc.at<float>(2));
      std::vector<float> ps;
      ps.reserve(3);
      ps.push_back(mbf / disp * (((float)kp.pt.x - cx) / fx));
      ps.push_back(mbf / disp * (((float)kp.pt.y - cy) / fy));
      ps.push_back(mbf / disp);
      zs.push_back(mbf / disp);
      std::unique_lock<std::mutex> lck(mutexPoints);
      posMonoInit_.push_back(pm);
      posStereoInit_.push_back(ps);
    }

    if (posMonoInit_.size() < 20)
      return 1;
    double filtering_time = ((double)cv::getTickCount());
    std::sort(zs.begin(), zs.end());

    SmootherMLS smls(1, zs[zs.size() / 2] / 5);
    std::vector<int> notOutliers = smls.outlierRemovalRadius(posStereoInit_);
    posMono_.reserve(posStereoInit_.size());
    posStereo_.reserve(posStereoInit_.size());
    if (notOutliers.size() < 20)
      return 1;
    for (uint i(0); i < notOutliers.size(); i++)
    {
      posMono_.push_back(posMonoInit_[notOutliers[i]]);
      posStereo_.push_back(posStereoInit_[notOutliers[i]]);
    }
    std::cout << "POINTS EVALUATED : " << posMono_.size() << "/" << notOutliers.size() << "/"
              << vpMPs.size() << std::endl;
    double Scale = GroundTruthTools::scaleMinMedian(posMono_, posStereo_);
    filtering_time =
        ((double)cv::getTickCount() - filtering_time) / cv::getTickFrequency();
    std::cout << "Time 2 " << filtering_time << "ms " << std::endl;
    std::cout << "Scale Corr stereo : " << Scale << std::endl;
    return Scale;
  }

  double GroundTruthFrame::Estimate3DLocalMap(Map *map, const double &s)
  {
    std::vector<float> Error;
    Error.reserve(posMono_.size());
    int count(0);
    double acc(0.0);
    mvLocalMapPoints.clear();
    mvStereoMapPoints.clear();
    std::unique_lock<std::mutex> lck(mutexPoints);

    for (size_t i(0); i < posMono_.size(); i++)
    {
      double er = sqrt(pow(posStereo_[i][0] - s * posMono_[i][0], 2) +
                       pow(posStereo_[i][1] - s * posMono_[i][1], 2) +
                       pow(posStereo_[i][2] - s * posMono_[i][2], 2));
      Error.push_back(er);
      acc += er;
      count++;
    }

    double sum(0.0);
    std::accumulate(Error.begin(), Error.end(), sum);
    double invc = 1 / ((double)count);
    std::cout << "Mean Error Surf : " << acc * invc << " " << Error.size() << " "
              << posMono_.size() << std::endl;
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(5)
        << uint(this->mTimeStamp);
    std::string name("ErrorGTs" + out.str() + ".txt");
    GroundTruthTools::saveResults(Error, name);
    return acc * invc;
  }

  std::vector<std::vector<float>> GroundTruthFrame::getPosMono()
  {
    std::unique_lock<std::mutex> lck(mutexPoints);
    return posMono_;
  }

  std::vector<std::vector<float>> GroundTruthFrame::getPosStereo()
  {
    std::unique_lock<std::mutex> lck(mutexPoints);
    return posStereo_;
  }
} // namespace defSLAM
