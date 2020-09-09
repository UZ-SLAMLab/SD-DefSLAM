#include "DefKLTLocalMapping.h"
#include "DefKeyFrame.h"
#include "DefMapDrawer.h"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "GroundTruthKeyFrame.h"
#include "NormalEstimator.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "PolySolver.h"
#include "SchwarpDatabase.h"
#include "ShapeFromNormals.h"
#include "SurfaceRegistration.h"
#include <Converter.h>
#include <chrono>
#include <ctime>
#include <numeric>
#include <stdio.h>
#include <unistd.h>

namespace defSLAM
{
  DefKLTLocalMapping::DefKLTLocalMapping(Map *pMap,
                                         const string &strSettingPath)
      : DefLocalMapping(pMap, strSettingPath)
  {
  }

  DefKLTLocalMapping::DefKLTLocalMapping(Map *pMap,
                                         const SettingsLoader &settingLoader)
      : DefLocalMapping(pMap, settingLoader)
  {
  }

  void DefKLTLocalMapping::CreateNewMapPoints()
  {
    cv::Mat Twc = referenceKF_->GetPoseInverse();
    size_t nval = referenceKF_->mvKeysUn.size();
    int cols = referenceKF_->imGray.cols;
    cv::Mat mask(referenceKF_->imGray.rows, referenceKF_->imGray.cols,
                 CV_8UC1, cv::Scalar(0));
    int const max_BINARY_value = 255;

    for (size_t i = 0; i < nval; i++)
    {
      MapPoint *pMP = referenceKF_->GetMapPoint(i);
      if (pMP)
      {
        if (pMP->isBad())
          continue;
        mask.at<char>(referenceKF_->mvKeysUn[i].pt.y,
                      referenceKF_->mvKeysUn[i].pt.x) = 255;
      }
    }
    cv::Mat kernel;
    int kernel_size = cols / 10;
    int ddepth = -1;
    cv::Point anchor(-1, -1);
    double delta;
    delta = 0;
    kernel = cv::Mat::ones(kernel_size, kernel_size, CV_32F);

    cv::filter2D(mask, mask, ddepth, kernel, anchor, delta, cv::BORDER_DEFAULT);
    double threshold_value = 1;
    //     1: Binary Inverted
    cv::threshold(mask, mask, threshold_value, max_BINARY_value, 0);
    uint newPoints(0);

    for (size_t i = 0; i < nval; i++)
    {
      MapPoint *pMP = referenceKF_->GetMapPoint(i);
      cv::Vec3b rgb = this->referenceKF_->RGBimage.at<cv::Vec3b>(
          this->referenceKF_->mvKeys[i].pt);

      if (pMP)
      {
        if (pMP->isBad())
          continue;
        DefMapPoint *defMP = static_cast<DefMapPoint *>(pMP);
        cv::Vec3f x3c;

        static_cast<DefKeyFrame *>(referenceKF_)
            ->surface->get3DSurfacePoint(i, x3c);

        cv::Mat x3ch(4, 1, CV_32F);
        x3ch.at<float>(0, 0) = x3c(0);
        x3ch.at<float>(1, 0) = x3c(1);
        x3ch.at<float>(2, 0) = x3c(2);
        x3ch.at<float>(3, 0) = 1;

        cv::Mat x3wh(4, 1, CV_32F);
        x3wh = Twc * x3ch;
        cv::Mat x3w(3, 1, CV_32F);
        x3w.at<float>(0, 0) = x3wh.at<float>(0, 0);
        x3w.at<float>(1, 0) = x3wh.at<float>(1, 0);
        x3w.at<float>(2, 0) = x3wh.at<float>(2, 0);
        cv::Mat X3Do = static_cast<DefMapPoint *>(pMP)
                           ->PosesKeyframes[referenceKF_]
                           .clone();

        if (X3Do.empty())
        {
          pMP->SetWorldPos(x3w);
          defMP->lastincorporasion = false;
          continue;
        }
        pMP->SetWorldPos(x3w);
      }
      else
      {
        const auto &kpt = referenceKF_->mvKeysUn[i].pt;
        if (uint8_t(mask.at<char>(kpt.y, kpt.x)) > 125)
        {
          continue;
        }
        cv::Vec3f x3c;
        static_cast<DefKeyFrame *>(referenceKF_)
            ->surface->get3DSurfacePoint(i, x3c);

        cv::Mat x3ch(4, 1, CV_32F);
        x3ch.at<float>(0, 0) = x3c(0);
        x3ch.at<float>(1, 0) = x3c(1);
        x3ch.at<float>(2, 0) = x3c(2);
        x3ch.at<float>(3, 0) = 1;

        cv::Mat x3wh(4, 1, CV_32F);
        x3wh = Twc * x3ch;
        cv::Mat x3w(3, 1, CV_32F);
        x3w.at<float>(0, 0) = x3wh.at<float>(0, 0);
        x3w.at<float>(1, 0) = x3wh.at<float>(1, 0);
        x3w.at<float>(2, 0) = x3wh.at<float>(2, 0);
        cv::Vec3f x3n;
        static_cast<DefKeyFrame *>(referenceKF_)
            ->surface->getNormalSurfacePoint(i, x3n);
        pMP = new DefMapPoint(x3w, referenceKF_, mpMap, referenceKF_->mvKeys[i].pt, x3n);
        //pMP = new DefMapPoint(x3w, referenceKF_, mpMap);

        pMP->AddObservation(referenceKF_, i);
        referenceKF_->addMapPoint(pMP, i);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();
        mpMap->addMapPoint(pMP);
        mlpRecentAddedMapPoints.push_back(pMP);
        auto tsize(cols / 10);
        auto ycrop = kpt.y - tsize / 2;
        auto xcrop = kpt.x - tsize / 2;
        if (ycrop < 0)
          ycrop = 0;
        if ((ycrop + tsize) > mask.rows)
          ycrop = mask.rows - tsize - 1;
        if (xcrop < 0)
          xcrop = 0;
        if ((xcrop + tsize) > mask.cols)
          xcrop = mask.cols - tsize - 1;
        auto crop = cv::Rect(xcrop, ycrop, tsize, tsize);
        cv::Mat Roi = mask(crop);
        Roi = cv::Scalar(255);
        newPoints++;
      }
    }
    std::cout << "Points Created : " << newPoints << std::endl;
  }

} // namespace defSLAM
