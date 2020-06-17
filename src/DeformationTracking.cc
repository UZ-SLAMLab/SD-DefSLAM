#include "DeformationTracking.h"
#include "DeformationOptimizer.h"
#include "GroundTruthFrame.h"
#include "Optimizer.h"

#include "TriangularMesh.h"
#include "DeformationORBmatcher.h"
#include "GroundTruthKeyFrame.h"
#include "MinMedianFilter.h"
#include "PnPsolver.h"
#include <DeformationLocalMapping.h>
#include <GroundTruthCalculator.h>

#include <DeformationMap.h>
#include <DeformationMapDrawer.h>

namespace defSLAM
{
  class DeformationLocalMapping;
  class DeformationMap;
  class DeformationMapDrawer;
  class Node;
  DeformationTracking::DeformationTracking(System *pSys, ORBVocabulary *pVoc,
                                           FrameDrawer *pFrameDrawer,
                                           MapDrawer *pMapDrawer, Map *pMap,
                                           KeyFrameDatabase *pKFDB,
                                           const string &strSettingPath,
                                           const int sensor, bool viewerOn)
      : Tracking(pSys, pVoc, pFrameDrawer, pMapDrawer, pMap, pKFDB,
                 strSettingPath, sensor, viewerOn)
  {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    RegLap = fSettings["Regularizer.laplacian"];
    RegInex = fSettings["Regularizer.Inextensibility"];
    RegTemp = fSettings["Regularizer.temporal"];
    double a = fSettings["Regularizer.LocalZone"];

    cout << endl
         << "Defomation tracking Parameters: " << endl;
    cout << "- Reg. Inextensibility: " << RegInex << endl;
    cout << "- Reg. Laplacian: " << RegLap << endl;
    cout << "- Reg. Temporal: " << RegTemp << endl;
    cout << "- Reg. LocalZone: " << a << endl;

    LocalZone = uint(a);

    ReliabilityThreshold = fSettings["Regularizer.Reliability"];

    ///-------------------------------
    mKLTtracker = LucasKanadeTracker(cv::Size(11, 11), 4, 10, 0.01, 1e-4);
    ///-------------------------------
  }

  void DeformationTracking::Track()
  {
    if (mState == NO_IMAGES_YET)
    {
      mState = NOT_INITIALIZED;
    }

    mLastProcessedState = mState;
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if (mState == NOT_INITIALIZED)
    {
      this->MonocularInitialization();
      if (mState != OK)
        return;
    }
    else
    {
      // System is initialized. Track Frame.
      bool bOK;
      // Initial camera pose estimation using motion model or relocalization (if
      // tracking is lost)
      if (!mbOnlyTracking)
      {
        mKLTtracker.AddFromKeyFrame(mpLastKeyFrame, mvKLTKeys, mvKLTMPs);
        mvKLTStatus.resize(mvKLTKeys.size(), true);

        std::set<MapPoint *> setklt;
        vector<MapPoint *> vectorklt;
        for (MapPoint *pMP : mvKLTMPs)
        {
          if (pMP)
          {
            setklt.insert(pMP);
            vectorklt.push_back(pMP);
          }
        }

        cout << "[LucasKanade-AddFromKeyFrame]: " << vectorklt.size() << " --- " << setklt.size() << endl;

        bOK = this->LocalisationAndMapping();
        mCurrentFrame->mpReferenceKF = mpReferenceKF;
        // If we have an initial estimation of the camera pose and matching. Track
        // the local map.
        if (bOK)
        {
          Timer timer;
          timer.start();
          if ((static_cast<DeformationLocalMapping *>(mpLocalMapper)
                   ->UpdateTemplate()))
          {
            mpReferenceKF =
                static_cast<DeformationMap *>(mpMap)->GetTemplate()->kf;
            Timer timer2;
            timer2.start();
            defSLAM::Optimizer::DefPoseOptimization(
                mCurrentFrame, mpMap, this->getRegLap(), this->getRegInex(), 0,
                LocalZone);
            timer2.stop();
            std::cout << "Time Insert SfN : " << timer2.getElapsedTimeInMilliSec()
                      << "ms " << std::endl;
            if (viewerOn)
            {
              static_cast<DeformationMapDrawer *>(mpMapDrawer)
                  ->UpdateTemplateAtRest();
            }
          }
          timer.stop();
          std::cout << "Time Insert Template : "
                    << timer.getElapsedTimeInMilliSec() << "ms " << std::endl;
          bOK = KLT_TrackLocalMap();
        }
      }
      else
      {

        bOK = this->OnlyLocalisation();
        // If we have an initial estimation of the camera pose and matching. Track
        // the local map.
        mCurrentFrame->mpReferenceKF = mpReferenceKF;

        // mbVO true means that there are few matches to MapPoints in the map. We
        // cannot retrieve
        // a local map and therefore we do not perform TrackLocalMap(). Once the
        // system relocalizes
        // the camera we will use the local map again.
        if (bOK && !mbVO)
          bOK = TrackLocalMap();
      }

      if (bOK)
      {
        mState = OK;

        // Update motion model
        if (!mLastFrame.mTcw.empty())
        {
          cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
          mLastFrame.GetRotationInverse().copyTo(
              LastTwc.rowRange(0, 3).colRange(0, 3));
          mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
          mVelocity = mCurrentFrame->mTcw * LastTwc;
        }
        else
          mVelocity = cv::Mat();

        if (viewerOn)
        {
          mpMapDrawer->SetCurrentCameraPose(mCurrentFrame->mTcw);
          mpFrameDrawer->Update(this);
          mpMapDrawer->UpdatePoints(mCurrentFrame);
          static_cast<DeformationMapDrawer *>(mpMapDrawer)->UpdateTemplate();
        }
        // Clean VO matches
        CleanMatches();
        // Erase Temporal points;
        EraseTemporalPoints();

// Check if we need to insert a new keyframe
#ifdef PARALLEL
        if (Tracking::NeedNewKeyFrame())
        {
          this->CreateNewKeyFrame();
        }
#else
        if ((mCurrentFrame->mnId % 10) < 1)
        {
          this->KLT_CreateNewKeyFrame();
        }
#endif

        // We allow points with high innovation (considererd outliers by the Huber
        // Function) pass to the new keyframe, so that bundle adjustment will
        // finally decide if they are outliers or not. We don't want next frame to
        // estimate its position with those points so we discard them in the
        // frame.
        for (size_t i = 0; i < mCurrentFrame->N; i++)
        {
          if (mCurrentFrame->mvpMapPoints[i] && mCurrentFrame->mvbOutlier[i])
          {
            mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint *>(nullptr);
          }
        }
      }

      else
      {
        mState = LOST;
        this->status << mCurrentFrame->mTimeStamp << " " << 1 << std::endl;
        if (viewerOn)
        {
          mpFrameDrawer->Update(this);
          mpMapDrawer->UpdatePoints(mCurrentFrame);
        }

        // Reset if the camera gets lost soon after initialization
        /*if((mpMap->KeyFramesInMap()<=3)||(!static_cast<DeformationMap*>(mpMap)->GetTemplate()))
      {*/
        cout << "Track lost soon after initialisation, reseting..." << endl;
        mpSystem->Reset();
        return;
        // }
      }

      if (!mCurrentFrame->mpReferenceKF)
        mCurrentFrame->mpReferenceKF = mpReferenceKF;
      mLastFrame = Frame(*mCurrentFrame);
    }

    if (!mCurrentFrame->mTcw.empty())
    {
      cv::Mat Tcr =
          mCurrentFrame->mTcw * mCurrentFrame->mpReferenceKF->GetPoseInverse();
      mlRelativeFramePoses.push_back(Tcr);
      mlpReferences.push_back(mpReferenceKF);
      mlFrameTimes.push_back(mCurrentFrame->mTimeStamp);
      mlbLost.push_back(mState == LOST);
    }
    else
    {
      // This can happen if tracking is lost
      mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
      mlpReferences.push_back(mlpReferences.back());
      mlFrameTimes.push_back(mlFrameTimes.back());
      mlbLost.push_back(mState == LOST);
    }
  }
  bool DeformationTracking::LocalisationAndMapping()
  {
    bool bOK = KLT_TrackWithMotionModel();
    return bOK;
  }

  bool DeformationTracking::NeedNewKeyFrame()
  {
    // Morlana's implementation of the keyframe insertion criteria
    if (mbOnlyTracking)
      return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
      return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last
    // relocalisation
    if (mCurrentFrame->mnId < mnLastRelocFrameId + mMaxFrames &&
        nKFs > mMaxFrames)
      return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if (nKFs <= 2)
      nMinObs = 2;

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be
    // potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose = 0;
    if (mSensor != System::MONOCULAR)
    {
      for (int i = 0; i < mCurrentFrame->N; i++)
      {
        if (mCurrentFrame->mvDepth[i] > 0 &&
            mCurrentFrame->mvDepth[i] < mThDepth)
        {
          if (mCurrentFrame->mvpMapPoints[i] && !mCurrentFrame->mvbOutlier[i])
            nTrackedClose++;
          else
            nNonTrackedClose++;
        }
      }
    }

    bool bNeedToInsertClose =
        false; //(nTrackedClose<200) && (nNonTrackedClose>50);

    // Thresholds
    float thRefRatio = 0.75f;
    if (nKFs < 2)
      thRefRatio = 0.4f;

    if (mSensor == System::MONOCULAR)
      thRefRatio = 0.9f;

    // Nodes viewed

    int nodesViewed = 0;

    std::set<Node *> Nodes =
        static_cast<DeformationMap *>(mpMap)->GetTemplate()->get_Nodes();
    for (std::set<Node *>::iterator it = Nodes.begin(); it != Nodes.end(); it++)
    {
      if (static_cast<Node *>(*it)->isLocal() ||
          static_cast<Node *>(*it)->isViewed())
      {
        nodesViewed++;
      }
    }

    float ratioViewed = (float)nodesViewed / Nodes.size();
    cout << "Total NODES: " << Nodes.size() << endl;
    cout << "nodes viewed: " << nodesViewed << endl;
    cout << "Ratio of viewed nodes: " << ratioViewed << endl;

    bNeedToInsertClose = ratioViewed < 0.80; // 0.4
    // bNeedToInsertClose = true;GrabImageMonocularGT

    bool badTemplate = mCurrentFrame->repError > 5.0;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe
    // insertion
    // const bool c1a = mCurrentFrame->mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame->mnId >= mnLastKeyFrameId + mMinFrames &&
                      bLocalMappingIdle);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of
    // visual odometry compared to map matches.
    const bool c2 = ((bNeedToInsertClose) && mnMatchesInliers > 30);
    // std::cout <<"INSERT KEYFRAME "<< ((c1a||c1b)&&c2)<< " " << c1a << " " <<
    // c1b << " " << c2 << std::endl;
    std::cout << "INSERT KEYFRAME " << ((c1b) && (c2 || badTemplate)) << " "
              << c1b << " " << c2 << " " << badTemplate << std::endl;
    if ((c1b) && (c2 || badTemplate))
    // if((c1a || c1b) && c2)
    {
      //   return true;

      if (badTemplate)
        cout << "Reprojection error is high: insert KF" << endl;
      // If the mapping accepts keyframes, insert keyframe.
      // Otherwise send a signal to interrupt BA
      if (bLocalMappingIdle)
      {
        return true;
      }
      else
      {
        mpLocalMapper->InterruptBA();
        if (mSensor != System::MONOCULAR)
        {
          if (mpLocalMapper->KeyframesInQueue() < 3)
            return true;
          else
            return false;
        }
        else
          return false;
      }
    }
    else
      return false;
  }

  void DeformationTracking::TrackDataset(
      const cv::Mat &im, std::vector<std::vector<double>> &matches,
      double timestamp, const cv::Mat &Tcw0)
  {
    mImGray = im;
    im.copyTo(mImRGB);
    if (mImGray.channels() == 3)
    {
      if (mbRGB)
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
      else
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
    }
    else if (mImGray.channels() == 4)
    {
      if (mbRGB)
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
      else
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
    }

    mCurrentFrame = new Frame(mImGray, timestamp, mK, Tcw0, mDistCoef, im);
    if (mState == NO_IMAGES_YET)
    {
      keyframe = new GroundTruthKeyFrame(*mCurrentFrame, mpMap, mpKeyFrameDB);
      mpMap->AddKeyFrame(keyframe);
      static_cast<TriangularMesh *>(
          static_cast<DeformationMap *>(mpMap)->GetTemplate())
          ->GetFacetTexture();
      mState = OK;
    }
    if (static_cast<DeformationMap *>(mpMap)->GetTemplate())
    {
      Optimizer::DefPoseOptimization(
          mCurrentFrame, mpMap, this->getRegLap(), this->getRegInex(),
          this->getRegTemp(), LocalZone);
    }
    else
    {
      ORB_SLAM2::Optimizer::PoseOptimization(mCurrentFrame);
    }
    if (viewerOn)
    {
      mpMapDrawer->SetCurrentCameraPose(mCurrentFrame->mTcw);
      mpFrameDrawer->Update(this);
      mpMapDrawer->UpdatePoints(mCurrentFrame);
    }
  }

  bool DeformationTracking::TrackLocalMap()
  {
    // We have an estimation of the camera pose and some map points tracked in the
    // frame. We retrieve the local map and try to find matches to points in the
    // local map.
    UpdateLocalMap();
    SearchLocalPoints();
    if (static_cast<DeformationMap *>(mpMap)->GetTemplate())
    {
      Optimizer::DefPoseOptimization(
          mCurrentFrame, mpMap, this->getRegLap(), this->getRegInex(),
          this->getRegTemp(), LocalZone);
    }
    else
    {
      ORB_SLAM2::Optimizer::PoseOptimization(mCurrentFrame);
    }
    // Optimize Pose
    mnMatchesInliers = 0;
    int mnMatchesOutliers(0);
    int DefnToMatchLOCAL(0);
    for (int i = 0; i < mCurrentFrame->N; i++)
    {
      if (mCurrentFrame->mvpMapPoints[i])
      {
        if (!mCurrentFrame->mvbOutlier[i])
        {
          mCurrentFrame->mvpMapPoints[i]->IncreaseFound();
          if (!mbOnlyTracking)
          {
            if (mCurrentFrame->mvpMapPoints[i]->Observations() > 0)
            {
              mnMatchesInliers++;
              if (static_cast<DeformationMapPoint *>(
                      mCurrentFrame->mvpMapPoints[i])
                      ->getFacet())
                DefnToMatchLOCAL++;
            }
          }
          else
            mnMatchesInliers++;
        }
        else
        {
          mnMatchesOutliers++;
        }
      }
    }
    auto points = mpMap->GetReferenceMapPoints();
    auto numberLocalMapPoints(0);
    for (auto pMP : points)
    {
      if (pMP)
      {
        if (pMP->isBad())
          continue;
        if (static_cast<DeformationMapPoint *>(pMP)->getFacet())
          if (mCurrentFrame->isInFrustum(pMP, 0.5))
          {
            numberLocalMapPoints++;
          }
      }
    }
    // Optimize Pose
    int observedFrame(0);
    int mI(0);
    int mO(0);
    for (int i = 0; i < mCurrentFrame->N; i++)
    {
      if (mCurrentFrame->mvpMapPoints[i])
      {
        if (mCurrentFrame->mvpMapPoints[i]->isBad())
          continue;
        observedFrame++;
        if (!mCurrentFrame->mvbOutlier[i])
        {
          mI++;
        }
        else
        {
          mO++;
        }
      }
    }

    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(5)
        << uint(mCurrentFrame->mTimeStamp);
    std::cout << out.str() << " " << mI << " " << mO << " "
              << numberLocalMapPoints << std::endl;
    this->matches << out.str() << " " << mI << " " << mO << " "
                  << numberLocalMapPoints << std::endl;
    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if (mCurrentFrame->mnId < mnLastRelocFrameId + mMaxFrames &&
        mnMatchesInliers < 20)
      return false;

    if (mnMatchesInliers < 10)
      return false;
    else
      return true;
  }

  bool DeformationTracking::TrackWithMotionModel()
  {
    DeformationORBmatcher Defmatcher(0.9, false);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame->SetPose(mLastFrame.mTcw);

    fill(mCurrentFrame->mvpMapPoints.begin(), mCurrentFrame->mvpMapPoints.end(),
         static_cast<MapPoint *>(nullptr));

    // Project points seen in previous frame
    int th(20);
    Timer timer;
    timer.start();
    int nmatches = Defmatcher.SearchByProjection(*mCurrentFrame, mLastFrame, th,
                                                 mSensor == System::MONOCULAR);

    std::cout << "POINTS matched:" << nmatches << std::endl;
    timer.stop();

    // If few matches, uses a wider window search
    if (nmatches /* +nmatches2 */ < 20)
    {
      fill(mCurrentFrame->mvpMapPoints.begin(), mCurrentFrame->mvpMapPoints.end(),
           static_cast<MapPoint *>(nullptr));
      nmatches = Defmatcher.SearchByProjection(*mCurrentFrame, mLastFrame, th + 5,
                                               mSensor == System::MONOCULAR);
    }
    // std::cout << "mnMatches 2 Motion: " << nmatches << std::endl;

    if (nmatches < 15)
      return false;
    return true;

    /// Optimize frame pose with all matches with a rigid model to initialize the
    /// pose of the camera
    Optimizer::PoseOptimization(mCurrentFrame, myfile);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame->N; i++)
    {
      if (mCurrentFrame->mvpMapPoints[i])
      {
        if (mCurrentFrame->mvbOutlier[i])
        {
          MapPoint *pMP = mCurrentFrame->mvpMapPoints[i];
          mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint *>(nullptr);
          mCurrentFrame->mvbOutlier[i] = false;
          pMP->mbTrackInView = false;
          pMP->mnLastFrameSeen = mCurrentFrame->mnId;
          nmatches--;
        }
        else if (mCurrentFrame->mvpMapPoints[i]->Observations() > 0)
          nmatchesMap++;
      }
    }

    if (mbOnlyTracking)
    {
      mbVO = nmatchesMap < 10;
      return nmatches > 20;
    }

    return nmatchesMap >= 10;
  }

  void DeformationTracking::UpdateLastFrame()
  {
    // Update pose according to reference keyframe
    KeyFrame *pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr * pRef->GetPose());

    if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR ||
        !mbOnlyTracking)
      return;
  }

  void DeformationTracking::UpdateLocalPoints()
  {
    std::set<MapPoint *> mPall;
    for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(),
                                            itEndKF = mvpLocalKeyFrames.end();
         itKF != itEndKF; itKF++)
    {
      KeyFrame *pKF = *itKF;
      const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();
      for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(),
                                              itEndMP = vpMPs.end();
           itMP != itEndMP; itMP++)
      {
        MapPoint *pMP = *itMP;
        if (!pMP)
          continue;
        if (pMP->mnTrackReferenceForFrame == mCurrentFrame->mnId)
          continue;
        /* if (!static_cast<DeformationMapPoint *>(pMP)->getFacet())
         continue;*/
        if (!pMP->isBad())
        {
          pMP->mnTrackReferenceForFrame = mCurrentFrame->mnId;
          mPall.insert(pMP);
        }
      }
    }
    mvpLocalMapPoints.clear();
    mvpLocalMapPoints.resize(mPall.size());
    std::copy(mPall.begin(), mPall.end(), mvpLocalMapPoints.begin());
  }

  cv::Mat DeformationTracking::GrabImageMonocularGT(const cv::Mat &imRectLeft,
                                                    const cv::Mat &imRectRight,
                                                    const double &timestamp,
                                                    cv::Mat _mask)
  {
    mImGray = imRectLeft.clone();
    cv::Mat imGrayRight = imRectRight;
    imRectLeft.copyTo(mImRGB);
    if (_mask.empty())
    {
      cv::Mat __mask(imRectLeft.rows, imRectLeft.cols, CV_8UC1, cv::Scalar(255));
      _mask = __mask.clone();
    }

    if (mImGray.channels() == 3)
    {
      if (mbRGB)
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
      }
      else
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
      }
    }
    else if (mImGray.channels() == 4)
    {
      if (mbRGB)
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
      }
      else
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
        cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
      }
    }
    else
    {
      cv::cvtColor(imRectLeft, mImRGB, cv::COLOR_GRAY2RGB);
    }
    Timer timer;
    timer.start();

    std::cout << mImGray.size() << std::endl;
    int action = (mState == eTrackingState::NO_IMAGES_YET || mState == eTrackingState::NOT_INITIALIZED) ? 0 : 2;
    cout << "STATUS: " << mState << endl;
    mCurrentFrame = new GroundTruthFrame(
        mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef,
        mbf, mThDepth, imRectLeft, imGrayRight, _mask, action);

    this->Track();
    timer.stop();
    std::cout << "Frame time : " << timer.getElapsedTimeInMilliSec() << " ms "
              << std::endl;
    if ((mState == eTrackingState::OK) && (true))
    {
      Timer timers;
      timers.start();
      float scale =
          static_cast<GroundTruthFrame *>(mCurrentFrame)->Estimate3DScale(mpMap);
      scalefile << mCurrentFrame->mTimeStamp << " " << scale << std::endl;
      double error = static_cast<GroundTruthFrame *>(mCurrentFrame)
                         ->Estimate3DLocalMap(mpMap, scale);
      timers.stop();
      std::cout << "Matching (" << scale
                << "): " << timers.getElapsedTimeInMilliSec() << " ms"
                << std::endl;
      if (viewerOn)
      {
        mpMapDrawer->UpdatePoints(mCurrentFrame, scale);
        this->mpFrameDrawer->SetError(error / 1000);
      }
    }

    return mCurrentFrame->mTcw.clone();
  }

  cv::Mat DeformationTracking::GrabImageMonocularCTGT(const cv::Mat &imRectLeft,
                                                      const cv::Mat &imDepth,
                                                      const double &timestamp,
                                                      cv::Mat _mask)
  {
    mImGray = imRectLeft.clone();
    imRectLeft.copyTo(mImRGB);

    if (mImGray.channels() == 3)
    {
      if (mbRGB)
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
      }
      else
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
      }
    }
    else if (mImGray.channels() == 4)
    {
      if (mbRGB)
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
      }
      else
      {
        cv::cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
      }
    }
    else
    {
      cv::cvtColor(imRectLeft, mImRGB, cv::COLOR_GRAY2RGB);
    }
    Timer timer;
    timer.start();

    mCurrentFrame = new GroundTruthFrame(
        mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef,
        mbf, mThDepth, imRectLeft, imDepth, true, _mask);

    this->Track();

    if ((mState == eTrackingState::OK) && (true))
    {
      Timer timers;
      timers.start();
      float scale =
          static_cast<GroundTruthFrame *>(mCurrentFrame)->Estimate3DScale(mpMap);
      scalefile << mCurrentFrame->mTimeStamp << " " << scale << std::endl;
      double error = static_cast<GroundTruthFrame *>(mCurrentFrame)
                         ->Estimate3DLocalMap(mpMap, scale);
      timers.stop();
      std::cout << "Matching (" << scale
                << "): " << timers.getElapsedTimeInMilliSec() << " ms"
                << std::endl;
      if (viewerOn)
      {
        mpMapDrawer->UpdatePoints(mCurrentFrame, scale);
        this->mpFrameDrawer->SetError(error);
      }
      // GroundTruthCalculator::CompareDeformationGT(&mCurrentFrame,timestamp);
    }
    timer.stop();
    std::cout << "Frame time : " << timer.getElapsedTimeInMilliSec() << " ms "
              << std::endl;

    return mCurrentFrame->mTcw.clone();
  }

  void DeformationTracking::MonocularInitialization()
  {
    /// Initialize the surface and the points in the surface considering a plane
    /// parallel to the camera plane
    if (mCurrentFrame->N > 100)
    {
      // Set Frame pose to the origin
      mCurrentFrame->SetPose(cv::Mat::eye(4, 4, CV_32F));

      // Create KeyFrame
      KeyFrame *pKFini =
          new GroundTruthKeyFrame(*mCurrentFrame, mpMap, mpKeyFrameDB);

      ///-----------------------------------
      cv::buildOpticalFlowPyramid(mCurrentFrame->ImGray, pKFini->imPyr, mKLTtracker.winSize, mKLTtracker.maxLevel);
      ///-----------------------------------

      // Insert KeyFrame in the map
      mpMap->AddKeyFrame(pKFini);
      // Create MapPoints with inliers and associate to KeyFrame
      for (size_t i = 0; i < size_t(mCurrentFrame->N); i++)
      {
        cv::KeyPoint kp = mCurrentFrame->mvKeysUn[i];

        cv::Mat x3D = (cv::Mat_<float>(3, 1)
                           << (kp.pt.x - mCurrentFrame->cx) / mCurrentFrame->fx,
                       (kp.pt.y - mCurrentFrame->cy) / mCurrentFrame->fy, 1);
        //MapPoint *pNewMP = new DeformationMapPoint(x3D, pKFini, mpMap);
        cv::Vec3f normal;
        normal(0) = 0;
        normal(1) = 0;
        normal(2) = -1;

        MapPoint *pNewMP = new DeformationMapPoint(x3D, pKFini, mpMap, mCurrentFrame->mvKeys[i].pt, normal);

        pNewMP->trackedByKLT = true;
        pNewMP->AddObservation(pKFini, i);
        pKFini->AddMapPoint(pNewMP, i);
        pNewMP->ComputeDistinctiveDescriptors();
        pNewMP->UpdateNormalAndDepth();
        mpMap->AddMapPoint(pNewMP);
        mCurrentFrame->mvpMapPoints[i] = pNewMP;
        pKFini->GetMapPoint(i);
      }

      ///------------------------------------
      mvKLTKeys = mCurrentFrame->mvKeys;
      mvKLTMPs = mCurrentFrame->mvpMapPoints;
      mvKLTStatus.resize(mCurrentFrame->N, true);

      mKLTtracker.SetReferenceImage(mCurrentFrame->ImGray, mvKLTKeys);
      ///------------------------------------

      double *Array;
      Array = new double[_NumberOfControlPointsU * _NumberOfControlPointsV];

      for (uint i(0); i < _NumberOfControlPointsU * _NumberOfControlPointsV;
           i++)
      {
        float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        Array[i] = 1;
      }
      BBS::bbs_t bbs;
      auto defkf = static_cast<DeformationKeyFrame *>(pKFini);
      bbs.umin = defkf->umin;
      bbs.umax = defkf->umax;
      bbs.nptsu = _NumberOfControlPointsU;
      bbs.nptsv = _NumberOfControlPointsV;
      bbs.vmax = defkf->vmax;
      bbs.vmin = defkf->vmin;
      bbs.valdim = 1;
      defkf->surface->SaveArray(Array, bbs);

      cout << "New map created with " << mpMap->MapPointsInMap() << " points"
           << endl;
      mLastFrame = Frame(*mCurrentFrame);
      mnLastKeyFrameId = uint(mCurrentFrame->mnId);
      mpLastKeyFrame = pKFini;
      mvpLocalKeyFrames.push_back(pKFini);
      mvpLocalMapPoints = mpMap->GetAllMapPoints();
      mpReferenceKF = pKFini;
      mCurrentFrame->mpReferenceKF = pKFini;
      mLastFrame.mpReferenceKF = pKFini;
      mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
      mpMap->mvpKeyFrameOrigins.push_back(pKFini);
      mpLocalMapper->InsertKeyFrame(pKFini);
      cv::Mat Tcr =
          mCurrentFrame->mTcw * mCurrentFrame->mpReferenceKF->GetPoseInverse();
      mlRelativeFramePoses.push_back(Tcr);
      // Initialize the SLAM
      static_cast<DeformationKeyFrame *>(pKFini)->assignTemplate();
      static_cast<DeformationMap *>(mpMap)->CreateTemplate(pKFini);
      std::cout << static_cast<DeformationMap *>(mpMap)->GetTemplate()->get_Nodes().size() << std::endl;

      if (viewerOn)
      {
        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame->mTcw);
        mpFrameDrawer->Update(this);
        mpMapDrawer->UpdatePoints(mCurrentFrame);
        static_cast<DeformationMapDrawer *>(mpMapDrawer)->UpdateTemplate();
        static_cast<DeformationMapDrawer *>(mpMapDrawer)->UpdateTemplateAtRest();
      }
      mState = OK;
    }
  }

  void DeformationTracking::CleanMatches()
  {
    for (int i = 0; i < mCurrentFrame->N; i++)
    {
      MapPoint *pMP = mCurrentFrame->mvpMapPoints[i];
      if (pMP)
        if (pMP->Observations() < 1)
        {
          mCurrentFrame->mvbOutlier[i] = false;
          mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint *>(nullptr);
        }
    }
  }

  void DeformationTracking::EraseTemporalPoints()
  {
    // Delete temporal MapPoints
    for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(),
                                    lend = mlpTemporalPoints.end();
         lit != lend; lit++)
    {
      MapPoint *pMP = *lit;
      delete pMP;
    }
    mlpTemporalPoints.clear();
  }

  void DeformationTracking::CreateNewKeyFrame()
  {
    if (!mpLocalMapper->SetNotStop(true))
      return;

    KeyFrame *pKF = new GroundTruthKeyFrame(*mCurrentFrame, mpMap, mpKeyFrameDB);
    mCurrentFrame->mpReferenceKF = mpReferenceKF;
    mpLocalMapper->InsertKeyFrame(pKF);
    mpLocalMapper->SetNotStop(false);
    mnLastKeyFrameId = mCurrentFrame->mnId;
    mpLastKeyFrame = pKF;
  }

  bool DeformationTracking::KLT_TrackWithMotionModel()
  {
    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame->SetPose(mLastFrame.mTcw);

    int toTrack = 0;

    for (size_t i = 0; i < mvKLTMPs.size(); i++)
    {
      if (mvKLTMPs[i])
      {
        MapPoint *pMP = mvKLTMPs[i];
        if (!static_cast<DeformationMapPoint *>(pMP)->getFacet())
          mvKLTStatus[i] = false;
        if (mvKLTStatus[i])
          toTrack++;
      }
    }

    //KLT_UpdateSeeds();

    int nmatches = mKLTtracker.PRE_Track(mCurrentFrame->ImGray, mvKLTKeys, mvKLTStatus, true, 0.85);

    for (size_t i = 0; i < mvKLTMPs.size(); i++)
    {
      if (!mvKLTStatus[i])
        continue;
      if (mvKLTKeys[i].pt.x < 20 || mvKLTKeys[i].pt.x > mCurrentFrame->ImGray.cols - 20 ||
          mvKLTKeys[i].pt.y < 20 || mvKLTKeys[i].pt.y > mCurrentFrame->ImGray.rows - 20)
      {
        mvKLTStatus[i] = false;
      }
      if (mCurrentFrame->_mask.at<uchar>(mvKLTKeys[i].pt.y, mvKLTKeys[i].pt.x) < 125)
        mvKLTStatus[i] = false;
      /*if(mvKLTMPs[i] && !static_cast<DeformationMapPoint *>(mvKLTMPs[i])->getFacet()){
            mvKLTStatus[i] = false;
        }*/
    }

    cout << "[KLT_TrackWithMotionModel]: points tracked by KLT: " << nmatches << " of " << toTrack << endl;

    mCurrentFrame->SetTrackedPoints(mvKLTKeys, mvKLTStatus, mvKLTMPs);

    std::set<MapPoint *> setM;

    for (MapPoint *pMP : mCurrentFrame->mvpMapPoints)
    {
      setM.insert(pMP);
    }

    cout << "[KLT_TrackWithMotionModel]: " << mCurrentFrame->mvpMapPoints.size() << " --- " << setM.size() << endl;

    std::set<MapPoint *> setklt;
    vector<MapPoint *> vectorklt;
    for (MapPoint *pMP : mvKLTMPs)
    {
      if (pMP)
      {
        setklt.insert(pMP);
        vectorklt.push_back(pMP);
      }
    }

    cout << "[LucasKanade-MotionModel]: " << vectorklt.size() << " --- " << setklt.size() << endl;

    if (nmatches < 15)
      return false;

    return true;
  }

  void DeformationTracking::KLT_UpdateSeeds()
  {
    const cv::Mat Rcw = mCurrentFrame->mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = mCurrentFrame->mTcw.rowRange(0, 3).col(3);

    float fx = mK.at<float>(0, 0), fy = mK.at<float>(1, 1);
    float cx = mK.at<float>(0, 2), cy = mK.at<float>(1, 2);

    const float k1 = mDistCoef.at<float>(0);
    const float k2 = mDistCoef.at<float>(1);
    const float p1 = mDistCoef.at<float>(2);
    const float p2 = mDistCoef.at<float>(3);
    const float k3 = mDistCoef.at<float>(4);

    for (size_t i = 0; i < mvKLTStatus.size(); i++)
    {
      if (mvKLTMPs[i])
      {
        MapPoint *pMP = mvKLTMPs[i];

        cv::Mat x3Dw = pMP->GetWorldPos();
        cv::Mat x3Dc = Rcw * x3Dw + tcw;

        // Project in image and check it is not outside
        const float x_ = x3Dc.at<float>(0) / x3Dc.at<float>(2);
        const float y_ = x3Dc.at<float>(1) / x3Dc.at<float>(2);

        float r2 = x_ * x_ + y_ * y_;

        float x__ = x_ * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2) + 2 * p1 * x_ * y_ + p2 * (r2 + 2 * x_ * x_);
        float y__ = y_ * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2) + p1 * (r2 + 2 * y_ * y_) + 2 * p2 * x_ * y_;

        const float u = fx * x__ + cx;
        const float v = fy * y__ + cy;

        if (u < 0 || u >= (float)mCurrentFrame->ImGray.cols ||
            v < 0 || v >= (float)mCurrentFrame->ImGray.rows)
        {
          mvKLTStatus[i] = false;
          continue;
        }

        mvKLTKeys[i].pt.x = u;
        mvKLTKeys[i].pt.y = v;

        //Set predicted octave
        const cv::Mat PO = x3Dw - mCurrentFrame->mOw;
        const float dist = cv::norm(PO);

        const int predictedOctave = pMP->PredictScale(dist, mCurrentFrame);

        mvKLTKeys[i].octave = predictedOctave;

        mvKLTStatus[i] = true;
      }
    }
  }

  void DeformationTracking::KLT_CreateNewKeyFrame()
  {
    if (!mpLocalMapper->SetNotStop(true))
      return;

    cout << "NEW KEYFRAME" << endl;
    //Update KLT vectors (at this point, a Frame only contains tracked KeyPoints with a MapPoint associated)
    for (auto pMP : mvKLTMPs)
    {
      if (pMP)
        pMP->trackedByKLT = false;
      //pMP->trackedByKLT = false;
    }
    mvKLTKeys = mCurrentFrame->mvKeys;
    mvKLTMPs = mCurrentFrame->mvpMapPoints;
    //mvKLTStatus.clear();// vector<bool>().swap(mvKLTStatus);
    mvKLTStatus.resize(mvKLTKeys.size(), true);
    /*for(auto pMP : mvKLTMPs) {
        if(pMP) pMP->trackedByKLT = true;
    }*/
    for (size_t i = 0; i < mvKLTMPs.size(); i++)
    {
      mvKLTStatus[i] = true;
      //if(mvKLTMPs[i]) mvKLTMPs[i]->trackedByKLT = true;
      mvKLTMPs[i]->trackedByKLT = true;
    }

    //Set new reference image for KLT
    mKLTtracker.SetReferenceImage(mCurrentFrame->ImGray, mvKLTKeys);

    //Create new KeyFrame
    KeyFrame *pKF = new GroundTruthKeyFrame(*mCurrentFrame, mpMap, mpKeyFrameDB);

    //Set image pyramid
    pKF->imPyr = vector<cv::Mat>(mKLTtracker.refPyr.size());
    for (int i = 0; i < mKLTtracker.refPyr.size(); i++)
    {
      pKF->imPyr[i] = mKLTtracker.refPyr[i].clone();
    }

    mCurrentFrame->mpReferenceKF = mpReferenceKF;
    mpLocalMapper->InsertKeyFrame(pKF);
    mpLocalMapper->SetNotStop(false);
    mnLastKeyFrameId = mCurrentFrame->mnId;
    mpLastKeyFrame = pKF;
  }

  bool DeformationTracking::KLT_TrackLocalMap()
  {
    // We have an estimation of the camera pose and some map points tracked in the
    // frame. We retrieve the local map and try to find matches to points in the
    // local map.
    UpdateLocalMap();
    int basePoints = KLT_SearchLocalMapPoints();
    //int basePoints = mCurrentFrame->N;

    if (static_cast<DeformationMap *>(mpMap)->GetTemplate())
    {
      Optimizer::DefPoseOptimization(
          mCurrentFrame, mpMap, this->getRegLap(), this->getRegInex(),
          this->getRegTemp(), LocalZone);
    }
    else
    {
      ORB_SLAM2::Optimizer::PoseOptimization(mCurrentFrame);
    }

    // Optimize Pose
    mnMatchesInliers = 0;
    int mnMatchesOutliers(0);
    int DefnToMatchLOCAL(0);
    for (int i = 0; i < mCurrentFrame->N; i++)
    {
      if (mCurrentFrame->mvpMapPoints[i])
      {
        if (!mCurrentFrame->mvbOutlier[i])
        {
          mCurrentFrame->mvpMapPoints[i]->IncreaseFound();
          if (!mbOnlyTracking)
          {
            if (mCurrentFrame->mvpMapPoints[i]->Observations() > 0)
            {
              mnMatchesInliers++;
              if (static_cast<DeformationMapPoint *>(
                      mCurrentFrame->mvpMapPoints[i])
                      ->getFacet())
                DefnToMatchLOCAL++;
            }
          }
          else
            mnMatchesInliers++;
        }
        else
        {
          mnMatchesOutliers++;
        }
      }
    }

    vector<cv::KeyPoint> vAllGoodKeys, vNewKLTKeys;
    vector<MapPoint *> vAllGoodMps, vNewKLTMPs;
    vector<bool> vGood;
    for (size_t i = 0; i < mCurrentFrame->N; i++)
    {
      if (mCurrentFrame->mvpMapPoints[i])
      {
        if (!mCurrentFrame->mvbOutlier[i] && mCurrentFrame->isInFrustum(mCurrentFrame->mvpMapPoints[i], 0.5))
        {
          vAllGoodKeys.push_back(mCurrentFrame->mvKeys[i]);
          vAllGoodMps.push_back(mCurrentFrame->mvpMapPoints[i]);
          vGood.push_back(true);

          if (i >= basePoints)
          {
            vNewKLTKeys.push_back(mCurrentFrame->mvKeys[i]);
            vNewKLTMPs.push_back(mCurrentFrame->mvpMapPoints[i]);
          }
        }
      }
    }

    //Update KLT
    mKLTtracker.AddPointsFromMapPoints(vNewKLTMPs, vNewKLTKeys, mImGray, mvKLTMPs, mvKLTKeys);

    //Update Frame
    mCurrentFrame->SetTrackedPoints(vAllGoodKeys, vGood, vAllGoodMps);

    std::set<MapPoint *> setklt;
    vector<MapPoint *> vectorklt;
    for (MapPoint *pMP : mvKLTMPs)
    {
      if (pMP)
      {
        setklt.insert(pMP);
        vectorklt.push_back(pMP);
      }
    }

    cout << "[LucasKanade-LocalMap]: " << vectorklt.size() << " --- " << setklt.size() << endl;

    auto points = mpMap->GetReferenceMapPoints();
    auto numberLocalMapPoints(0);
    set<MapPoint *> refPts;
    for (auto pMP : points)
    {
      if (pMP)
      {
        if (pMP->isBad())
          continue;
        refPts.insert(pMP);
        if (static_cast<DeformationMapPoint *>(pMP)->getFacet())
          if (mCurrentFrame->isInFrustum(pMP, 0.5))
          {
            numberLocalMapPoints++;
          }
      }
    }

    int notInRef = 0;
    for (MapPoint *pMP : mCurrentFrame->mvpMapPoints)
    {
      if (pMP && !pMP->isBad())
      {
        if (refPts.count(pMP) == 0)
          notInRef++;
      }
    }

    cout << "Points not in ref: " << notInRef << endl;

    int notInFrustrum = 0;
    for (MapPoint *pMP : mCurrentFrame->mvpMapPoints)
    {
      if (pMP && !pMP->isBad())
      {
        if (!mCurrentFrame->isInFrustum(pMP, 0.5))
          notInFrustrum++;
      }
    }

    cout << "Points not in frustrum: " << notInFrustrum << endl;

    // Optimize Pose
    int observedFrame(0);
    int mI(0);
    int mO(0);
    std::vector<MapPoint *> vect;
    std::set<MapPoint *> setM;
    for (int i = 0; i < mCurrentFrame->N; i++)
    {
      if (mCurrentFrame->mvpMapPoints[i])
      {
        if (mCurrentFrame->mvpMapPoints[i]->isBad())
          continue;
        vect.push_back(mCurrentFrame->mvpMapPoints[i]);
        setM.insert(mCurrentFrame->mvpMapPoints[i]);
        observedFrame++;
        if (!mCurrentFrame->mvbOutlier[i])
        {
          mI++;
        }
        else
        {
          mO++;
        }
      }
    }

    std::cout << "UEJAAA :" << vect.size() << " " << setM.size() << std::endl;
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(5)
        << uint(mCurrentFrame->mTimeStamp);
    std::cout << out.str() << " " << mI << " " << mO << " "
              << numberLocalMapPoints << " ---- " << points.size() << std::endl;
    this->matches << out.str() << " " << mI << " " << mO << " "
                  << numberLocalMapPoints << std::endl;

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if (mCurrentFrame->mnId < mnLastRelocFrameId + mMaxFrames &&
        mnMatchesInliers < 20)
      return false;

    if (mnMatchesInliers < 10)
      return false;
    else
      return true;
  }

  int DeformationTracking::KLT_SearchLocalMapPoints()
  {
    int toReturn = mCurrentFrame->N;

    // Do not search map points already matched
    for (vector<MapPoint *>::iterator vit = mCurrentFrame->mvpMapPoints.begin(), vend = mCurrentFrame->mvpMapPoints.end(); vit != vend; vit++)
    {
      MapPoint *pMP = *vit;
      if (pMP && !pMP->trackedByKLT)
      {
        if (pMP->isBad())
        {
          *vit = static_cast<MapPoint *>(NULL);
        }
        else
        {
          pMP->IncreaseVisible();
          pMP->mnLastFrameSeen = mCurrentFrame->mnId;
          pMP->mbTrackInView = false;
        }
      }
    }

    int nToMatch = 0;

    vector<vector<cv::Mat>> vPatches, vGrad;
    vector<vector<float>> vMean, vMean2;
    vector<cv::KeyPoint> vNextPts, vPrevPts;
    vector<bool> bStatus;
    vector<MapPoint *> vMPs;
    vector<cv::Mat> vH;

    cv::Mat Kinv = mK.inv();

    // Project points in frame and check its visibility
    for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++)
    {
      MapPoint *pMP = *vit;
      if (pMP->mnLastFrameSeen == mCurrentFrame->mnId || pMP->trackedByKLT)
        continue;
      if (pMP->isBad())
        continue;
      // Project (this fills MapPoint variables for matching)
      if (mCurrentFrame->isInFrustum(pMP, 0.5))
      {
        if (pMP->mvPatch.empty())
        {
          pMP->KLT_ComputeMatrices(pMP->mObs);
        }
        pMP->IncreaseVisible();
        nToMatch++;

        vPatches.push_back(pMP->mvPatch);
        vGrad.push_back(pMP->mvGrad);
        vMean.push_back(pMP->mvMean);
        vMean2.push_back(pMP->mvMean2);

        cv::KeyPoint kp(pMP->mTrackProjX, pMP->mTrackProjY, 1, 1, 1, pMP->mnTrackScaleLevel);
        cv::KeyPoint kpR(pMP->mObs.x, pMP->mObs.y, 1, 1, 1, pMP->mnTrackScaleLevel);
        vNextPts.push_back(kp);
        vPrevPts.push_back(kpR);
        vMPs.push_back(pMP);

        ///-------------------------------
        ///     Compute Homography
        ///-------------------------------
        //Get Rfk and tfk
        cv::Mat Twk = pMP->pKF->GetPoseInverse();
        cv::Mat Tfk = mCurrentFrame->mTcw * Twk;

        cv::Mat Rfk = Tfk.rowRange(0, 3).colRange(0, 3);
        cv::Mat tfk = Tfk.rowRange(0, 3).col(3);

        //Get normal vector
        //We compute normal vector by unprojecting the image point in the KeyFrame
        cv::Mat Rk = pMP->pKF->GetRotation();
        cv::Mat n = Rk * pMP->GetNormal();
        cv::normalize(n, n);

        //Compute d
        cv::Mat Xk = pMP->pKF->GetRotation() * pMP->GetWorldPos() + pMP->pKF->GetTranslation();
        float d = cv::norm(Xk);

        //Finally, compute Homography matrix
        cv::Mat H = mK * (Rfk - tfk * n.t() / d) * Kinv;
        H = cv::Mat::eye(3, 3, CV_32F);

        vH.push_back(H.clone());
      }
    }

    if (nToMatch > 0)
    {
      LucasKanadeTracker kltLocalTracker = LucasKanadeTracker(cv::Size(11, 11), 1, 5, 0.1, 1e-4);
      //Use KLT to estimate Projected Local MapPoints
      int goodKLT = kltLocalTracker.TrackWithInfoWithHH(mCurrentFrame->ImGray, vNextPts, vPrevPts, bStatus, vPatches, vGrad, vMean, vMean2, vH, 0.75);

      for (size_t i = 0; i < vNextPts.size(); i++)
      {
        if (!bStatus[i])
          continue;
        if (vNextPts[i].pt.x < 20 || vNextPts[i].pt.x > mCurrentFrame->ImGray.cols - 20 ||
            vNextPts[i].pt.y < 20 || vNextPts[i].pt.y > mCurrentFrame->ImGray.rows - 20)
        {
          bStatus[i] = false;
        }
        if (mCurrentFrame->_mask.at<uchar>(vNextPts[i].pt.y, vNextPts[i].pt.x) < 125)
          bStatus[i] = false;
      }

      //Update current Frame with tracked local MapPoints
      mCurrentFrame->AppendTrackedPoints(vNextPts, bStatus, vMPs);
    }

    return toReturn;
  }
} // namespace defSLAM
