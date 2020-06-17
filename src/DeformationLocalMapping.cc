#include "DeformationLocalMapping.h"
#include "DeformationKeyFrame.h"
#include "DeformationMapDrawer.h"
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
#include "Timer.h"
#include <Converter.h>
#include <chrono>
#include <ctime>
#include <numeric>
#include <stdio.h>
#include <unistd.h>

float RandomFloat(float a, float b)
{
  float random = ((float)rand()) / (float)RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

namespace defSLAM
{
  DeformationLocalMapping::DeformationLocalMapping(Map *pMap, MapDrawer *mpDrawer,
                                                   const float bMonocular,
                                                   const string &strSettingPath)
      : LocalMapping(pMap, mpDrawer, bMonocular), CreatingTemplate(false),
        CreateTemplate(false), pointsToTemplate_(100), chiLimit_(0.07)
  {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    pointsToTemplate_ = fSettings["LocalMapping.pointsToTemplate"];
    chiLimit_ = fSettings["LocalMapping.chiLimit"];
    double reg_ = fSettings["LocalMapping.Schwarp.Regularizer"];
    bendingReg_ = fSettings["LocalMapping.Bending"];
    mpSchwarpDB = new SchwarpDatabase(reg_);
  }

  DeformationLocalMapping::~DeformationLocalMapping() { delete mpSchwarpDB; }

  void DeformationLocalMapping::Run()
  {

    mbFinished = false;

    while (1)
    {
      SetAcceptKeyFrames(false);
      insideTheLoop();
      if (Stop())
      {
        // Safe area to stop
        while (isStopped() && !CheckFinish())
        {
          usleep(3000);
        }
        if (CheckFinish())
          break;
      }

      ResetIfRequested();

      if (CheckFinish())
        break;

      SetAcceptKeyFrames(true);

      usleep(10000);
    }

    SetFinish();
  }

  void DeformationLocalMapping::insideTheLoop()
  {
    // Tracking will see that Local Mapping is busy
    // Check if there are keyframes in the queue
    if (CheckNewKeyFrames())
    {
      // Tracking will see that Local Mapping is busy
      Timer timer;
      Timer timer2;
      timer2.start();
      timer.start();
      // BoW conversion and insertion in Map
      this->ProcessNewKeyFrame();
      this->MapPointCulling();
      timer.stop();
      printf("ProcessNewKeyFrame Time taken: %.2fms\n",
             timer.getElapsedTimeInMilliSec());
      // Triangulate new MapPoints
      timer.start();
      this->CreateNewMapPoints();
      timer.stop();
      printf("CreateNewMapPoints Time taken: %.2fms\n",
             timer.getElapsedTimeInMilliSec());
      mbAbortBA = false;

      timer2.stop();
      printf("Total Time taken: %.2fms\n", timer2.getElapsedTimeInMilliSec());
    }
  }

  void DeformationLocalMapping::ResetIfRequested()
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbResetRequested)
    {
      static_cast<DeformationMap *>(mpMap)->clear();
      mpSchwarpDB->clear();
      CreatingTemplate = false;
      CreateTemplate = false;
      mlNewKeyFrames.clear();
      mlpRecentAddedMapPoints.clear();
      mbResetRequested = false;
    }
  }

  bool DeformationLocalMapping::UpdateTemplate()
  {
    unique_lock<mutex> lock(mMutexReset);
    if ((CreateTemplate) & (!stopRequested()))
    {
      Timer timer;
      timer.start();
      CreatingTemplate = true;
      std::unique_lock<std::mutex> M(
          static_cast<DeformationMap *>(mpMap)->MutexUpdating);
      static_cast<DeformationMap *>(mpMap)->clearTemplate();
      timer.start();
      this->CreatePoints();
      timer.stop();
      printf("New Points time taken: %.2fms\n", timer.getElapsedTimeInMilliSec());
      timer.start();
      static_cast<DeformationMap *>(mpMap)->CreateTemplate(KFtoTriangulate);
      timer.stop();
      std::cout << "timer map: " << timer.getElapsedTimeInMilliSec() << " ms. "
                << std::endl;
      static_cast<DeformationKeyFrame *>(KFtoTriangulate)->assignTemplate();
      CreatingTemplate = false;
      CreateTemplate = false;
      return true;
    }
    return false;
  }

  void DeformationLocalMapping::ProcessNewKeyFrame()
  {
    LocalMapping::ProcessNewKeyFrame();
    mpSchwarpDB->add(mpCurrentKeyFrame);
  }

  void DeformationLocalMapping::CreateNewMapPoints()
  {
    // When there is more than two keyframes, the non-rigid reconstruction starts.
    Timer timer;
    timer.start();
    {
      std::cout << "NORMAL ESTIMATOR IN - ";
      NormalEstimator NormalEstimator_(this->mpSchwarpDB);
      NormalEstimator_.ObtainK1K2();
      std::cout << " NORMAL ESTIMATOR OUT";
    }
    if (mpMap->GetAllKeyFrames().size() == 1)
    {
      KFtoTriangulate = mpMap->GetAllKeyFrames()[0];
    }
    timer.stop();
    printf("Normals Time taken: %.2fms\n", timer.getElapsedTimeInMilliSec());

    auto newTemplate = needNewTemplate();
    std::cout << "NEW template " << newTemplate << std::endl;
    KeyFrame *kfForTemplate;
    if (newTemplate)
    {
      std::cout << "NEW template " << std::endl;
      kfForTemplate = mpCurrentKeyFrame;
    }
    else
    {
      kfForTemplate = selectKeyframe();
    }

    if (!static_cast<DeformationKeyFrame *>(kfForTemplate)
             ->surface->EnoughNormals())
    {
      printf("Not enough normals /n");
      return;
    }
    {
      std::cout << "BENDING ESTIMATOR IN - ";
      timer.start();
      ShapeFromNormals SfN(kfForTemplate, mpSchwarpDB, bendingReg_);

      // Integrate the normals to recover the surface
      if (!SfN.InitialSolution())
      {
        printf("Not SfN sucessful");
        return;
      }
      std::cout << " BENDING ESTIMATOR OUT";
    }
    if (false)
    {
      float scale =
          static_cast<GroundTruthKeyFrame *>(kfForTemplate)->Estimate3DScale();
      std::cout << "Scale Error Keyframe : " << scale << " " << std::endl;
    }
    if (kfForTemplate != mpMap->GetAllKeyFrames()[0])
    {
      SurfaceRegistration SurfaceRegistration_(kfForTemplate, chiLimit_, true);
      bool wellRegistered = SurfaceRegistration_.Register();
      if (!wellRegistered)
      {
        printf("SurfaceRegistration not sucessful");
        return;
      }
    }
    KFtoTriangulate = kfForTemplate;

    timer.stop();
    NonRigidInitialisated = true;
    CreateTemplate = true;
  }

  bool DeformationLocalMapping::needNewTemplate()
  {
    int nval = this->mpCurrentKeyFrame->mvKeysUn.size();
    int cols = mpCurrentKeyFrame->imGray.cols;
    cv::Mat mask(mpCurrentKeyFrame->imGray.rows, mpCurrentKeyFrame->imGray.cols,
                 CV_8UC1, cv::Scalar(0));
    int const max_BINARY_value = 255;

    for (int i = 0; i < nval; i++)
    {
      MapPoint *pMP = mpCurrentKeyFrame->GetMapPoint(i);
      if (pMP)
      {
        if (pMP->isBad())
          continue;
        mask.at<char>(this->mpCurrentKeyFrame->mvKeysUn[i].pt.y,
                      this->mpCurrentKeyFrame->mvKeysUn[i].pt.x) = 255;
      }
    }
    cv::Mat kernel;
    int kernel_size = cols / 20;
    int ddepth = -1;
    cv::Point anchor(-1, -1);
    double delta;
    delta = 0;
    kernel = cv::Mat::ones(kernel_size, kernel_size, CV_32F);
    cv::filter2D(mask, mask, ddepth, kernel, anchor, delta, cv::BORDER_DEFAULT);
    double threshold_value = 1;
    cv::threshold(mask, mask, threshold_value, max_BINARY_value, 0);

    int newPoints(0);
    for (int i = 0; i < nval; i++)
    {
      MapPoint *pMP = mpCurrentKeyFrame->GetMapPoint(i);
      if (!pMP)
      {
        const auto &kpt = mpCurrentKeyFrame->mvKeysUn[i].pt;
        if (mask.at<char>(kpt.y, kpt.x))
        {
          continue;
        }
        newPoints++;
        auto tsize(cols / 20);
        auto ycrop = kpt.y - tsize / 2;
        auto xcrop = kpt.x - tsize / 2;
        if (xcrop < 0)
          xcrop = 0;
        if (ycrop < 0)
          ycrop = 0;
        if ((ycrop + tsize / 2) > mask.rows)
          ycrop = mask.rows - 1;
        if ((xcrop + tsize / 2) > mask.cols)
          xcrop = mask.cols - 1;
        auto crop = cv::Rect(xcrop, ycrop, tsize, tsize);
        cv::Mat Roi = mask(crop);
        Roi = cv::Scalar(255);
      }
    }
    // cv::imshow("Mask", mask);
    // cv::waitKey(10);
    bool createNewTemplate = (newPoints > pointsToTemplate_);
    std::cout << "Points potential : " << newPoints << "  " << pointsToTemplate_
              << std::endl;
    return createNewTemplate;
  }

  void DeformationLocalMapping::KeyFrameCulling()
  {
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees,
    // are seen in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<KeyFrame *> vpLocalKeyFrames =
        mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for (vector<KeyFrame *>::iterator vit = vpLocalKeyFrames.begin(),
                                      vend = vpLocalKeyFrames.end();
         vit != vend; vit++)
    {
      KeyFrame *pKF = *vit;
      if (pKF->mnId == 0)
        continue;
      const vector<MapPoint *> vpMapPoints = pKF->GetMapPointMatches();

      int nObs = 6;
      const int thObs = nObs;
      int nRedundantObservations = 0;
      int nMPs = 0;
      for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++)
      {
        MapPoint *pMP = vpMapPoints[i];
        if (pMP)
        {
          if (!pMP->isBad())
          {
            if (!mbMonocular)
            {
              if (pKF->mvDepth[i] > pKF->mThDepth || pKF->mvDepth[i] < 0)
                continue;
            }

            nMPs++;
            if (pMP->Observations() > thObs)
            {
              const int &scaleLevel = pKF->mvKeysUn[i].octave;
              const map<KeyFrame *, size_t> observations = pMP->GetObservations();
              int nObs = 0;
              for (map<KeyFrame *, size_t>::const_iterator
                       mit = observations.begin(),
                       mend = observations.end();
                   mit != mend; mit++)
              {
                KeyFrame *pKFi = mit->first;
                if (pKFi == pKF)
                  continue;

                const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                if (scaleLeveli <= scaleLevel + 1)
                {
                  nObs++;
                  if (nObs >= thObs)
                    break;
                }
              }
              if (nObs >= thObs)
              {
                nRedundantObservations++;
              }
            }
          }
        }
      }

      if (nRedundantObservations > 0.9 * nMPs)
      {
        this->mpSchwarpDB->erase(pKF);
        pKF->SetBadFlag();
      }
    }
  }

  void DeformationLocalMapping::CreatePoints()
  {
    cv::Mat Twc = KFtoTriangulate->GetPoseInverse();
    size_t nval = this->KFtoTriangulate->mvKeysUn.size();
    int cols = KFtoTriangulate->imGray.cols;
    cv::Mat mask(KFtoTriangulate->imGray.rows, KFtoTriangulate->imGray.cols,
                 CV_8UC1, cv::Scalar(0));
    int const max_BINARY_value = 255;

    for (int i = 0; i < nval; i++)
    {
      MapPoint *pMP = KFtoTriangulate->GetMapPoint(i);
      if (pMP)
      {
        if (pMP->isBad())
          continue;
        mask.at<char>(this->KFtoTriangulate->mvKeysUn[i].pt.y,
                      this->KFtoTriangulate->mvKeysUn[i].pt.x) = 255;
      }
    }
    cv::Mat kernel;
    int kernel_size = cols / 20;
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
      MapPoint *pMP = KFtoTriangulate->GetMapPoint(i);
      cv::Vec3b rgb = this->KFtoTriangulate->RGBimage.at<cv::Vec3b>(
          this->KFtoTriangulate->mvKeys[i].pt);
      int b, g, r;
      r = rgb(0);
      g = rgb(1);
      b = rgb(2);

      if (pMP)
      {
        if (pMP->isBad())
          continue;
        DeformationMapPoint *defMP = static_cast<DeformationMapPoint *>(pMP);
        cv::Vec3f x3c;

        static_cast<DeformationKeyFrame *>(KFtoTriangulate)
            ->surface->Get3DSurfacePoint(i, x3c);

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
        cv::Mat X3Do = static_cast<DeformationMapPoint *>(pMP)
                           ->PosesKeyframes[KFtoTriangulate]
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
        const auto &kpt = KFtoTriangulate->mvKeysUn[i].pt;
        if (mask.at<char>(kpt.y, kpt.x))
        {
          continue;
        }
        cv::Vec3f x3c;
        static_cast<DeformationKeyFrame *>(KFtoTriangulate)
            ->surface->Get3DSurfacePoint(i, x3c);

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
        static_cast<DeformationKeyFrame *>(KFtoTriangulate)
            ->surface->GetNormalSurfacePoint(i, x3n);
        pMP = new DeformationMapPoint(x3w, KFtoTriangulate, mpMap, KFtoTriangulate->mvKeys[i].pt, x3n);
        //pMP = new DeformationMapPoint(x3w, KFtoTriangulate, mpMap);

        pMP->AddObservation(KFtoTriangulate, i);
        KFtoTriangulate->AddMapPoint(pMP, i);

        pMP->ComputeDistinctiveDescriptors();
        static_cast<DeformationMapPoint *>(pMP)->SetRGB(r, g, b);
        pMP->UpdateNormalAndDepth();
        mpMap->AddMapPoint(pMP);
        mlpRecentAddedMapPoints.push_back(pMP);
        auto tsize(cols / 20);
        auto ycrop = kpt.y - tsize / 2;
        auto xcrop = kpt.x - tsize / 2;
        if (ycrop < 0)
          ycrop = 0;
        if ((ycrop + tsize / 2) > mask.rows)
          ycrop = mask.rows - 1;
        if (xcrop < 0)
          xcrop = 0;
        if ((xcrop + tsize / 2) > mask.cols)
          xcrop = mask.cols - 1;
        auto crop = cv::Rect(xcrop, ycrop, tsize, tsize);
        cv::Mat Roi = mask(crop);
        Roi = cv::Scalar(255);
        newPoints++;
      }
    }
    std::cout << "Points Created : " << newPoints << std::endl;
  }

  ORB_SLAM2::KeyFrame *DeformationLocalMapping::selectKeyframe()
  {
    size_t nval = this->mpCurrentKeyFrame->mvKeysUn.size();
    std::unordered_map<KeyFrame *, int> countKFMatches;

    for (size_t i = 0; i < nval; i++)
    {
      MapPoint *pMP = mpCurrentKeyFrame->GetMapPoint(i);
      if (pMP)
      {
        if (pMP->isBad())
          continue;
        KeyFrame *refkfi = pMP->GetReferenceKeyFrame();
        if (countKFMatches.count(refkfi) == 0)
          countKFMatches[refkfi] = 0;
        countKFMatches[refkfi]++;
      }
    }
    KeyFrame *refkfMaxPoints = mpCurrentKeyFrame;
    int CountPoints(0);
    for (auto &k : countKFMatches)
    {
      if (CountPoints < k.second)
      {
        refkfMaxPoints = k.first;
        CountPoints = k.second;
      }
    }

    return refkfMaxPoints;
  }
} // namespace defSLAM
