/*
 * This file is part of SD-DefSLAM
 * Copyright (C) 2020 Juan J. Gómez Rodríguez, Jose Lamarca Peiro, J. Morlana,
 *                    Juan D. Tardós and J.M.M. Montiel, University of Zaragoza
 *
 * This software is for internal use in the EndoMapper project.
 * Not to be re-distributed.
 */

#include "DefKLTTracking.h"
#include "DefOptimizer.h"
#include "GroundTruthFrame.h"
#include "Optimizer.h"

#include "TriangularMesh.h"
#include "DefORBmatcher.h"
#include "GroundTruthKeyFrame.h"
#include "MinMedianFilter.h"
#include "PnPsolver.h"
#include <DefLocalMapping.h>
#include <GroundTruthCalculator.h>

#include <DefMap.h>
#include <DefMapDrawer.h>

namespace defSLAM
{
  class DefLocalMapping;
  class DefMap;
  class DefMapDrawer;
  class Node;
  DefKLTTracking::DefKLTTracking(System *pSys, ORBVocabulary *pVoc,
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
    double SaveResults = fSettings["Viewer.SaveResults"];
    saveResults = bool(uint(SaveResults));
    double debug = fSettings["Debug.bool"];
    debugPoints = bool(uint(debug));
    cout << endl
         << "Defomation tracking Parameters: " << endl;
    cout << "- Reg. Inextensibility: " << RegInex << endl;
    cout << "- Reg. Laplacian: " << RegLap << endl;
    cout << "- Reg. Temporal: " << RegTemp << endl;
    cout << "- Reg. LocalZone: " << a << endl;

    LocalZone = uint(a);

    ReliabilityThreshold = fSettings["Regularizer.Reliability"];
    std::ofstream myfile("matches.txt");

    myfile.close();
    ///-------------------------------
    mKLTtracker = LucasKanadeTracker(cv::Size(11, 11), 4, 10, 0.01, 1e-4);
    ///-------------------------------
  }

  DefKLTTracking::DefKLTTracking(System *pSys, ORBVocabulary *pVoc,
                                 FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
                                 Map *pMap, KeyFrameDatabase *pKFDB,
                                 const SettingsLoader &settingLoader,
                                 const int sensor,
                                 bool viewerOn)
      : Tracking(pSys, pVoc, pFrameDrawer, pMapDrawer, pMap, pKFDB,
                 settingLoader, sensor, viewerOn)
  {
    RegLap = settingLoader.getregLap();
    RegInex = settingLoader.getregStreching();
    RegTemp = settingLoader.getregTemp();
    LocalZone = settingLoader.getLocalZone();
    ReliabilityThreshold = settingLoader.getreliabilityThreshold();
    saveResults = settingLoader.getSaveResults();
    debugPoints = settingLoader.getDebugPoints();

    mKLTtracker = LucasKanadeTracker(cv::Size(11, 11), 4, 10, 0.01, 1e-4);
  }

  void DefKLTTracking::Track()
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
        // If we have an initial estimation of the camera pose and matching. Track
        // the local map.
        if (mState == OK)
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

          if ((static_cast<DefLocalMapping *>(mpLocalMapper)
                   ->updateTemplate()))
          {
            mpReferenceKF =
                static_cast<DefMap *>(mpMap)->GetTemplate()->kf;
            defSLAM::Optimizer::DefPoseOptimization(
                mCurrentFrame, mpMap, this->getRegLap(), this->getRegInex(), 0,
                LocalZone);

            if (viewerOn)
            {
              static_cast<DefMapDrawer *>(mpMapDrawer)
                  ->updateTemplateAtRest();
            }
          }

          if (bOK)
          {
            bOK = TrackLocalMap();

            if (debugPoints)
              printCurrentPoints("DefSLAM: points post-KLT local map");
          }
        }
        else
        {
          // We recover the pose with DBoW and PnP
          // Take the old template associated with pKFreloc to continue processing
          bOK = relocalization();
          if (!bOK)
            cout << "Relocalization failed." << endl;
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

        if (debugPoints)
          printPointsWatchedByKeyframes("DefSLAM: points watched by KeyFrames");

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
          static_cast<DefMapDrawer *>(mpMapDrawer)->updateTemplate();
        }
        // Clean VO matches
        CleanMatches();
        // Erase Temporal points;
        EraseTemporalPoints();

        // Check if we need to insert a new keyframe
        if (DebugNeedNewKeyFrame())
        {
          this->CreateNewKeyFrame();
        }

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
        cout << "State LOST." << endl;
        this->status << mCurrentFrame->mTimeStamp << " " << 1 << std::endl;
        if (viewerOn)
        {
          mpFrameDrawer->Update(this);
          mpMapDrawer->UpdatePoints(mCurrentFrame);
        }

        if (mpMap->KeyFramesInMap() <= 5)
        {
          cout << "Track lost soon after initialisation, reseting..." << endl;
          mpSystem->Reset();
          return;
        }
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
      cout << "Tracking LOST." << endl;
      static_cast<DefMap *>(mpMap)->clearTemplate();

      mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
      mlpReferences.push_back(mlpReferences.back());
      mlFrameTimes.push_back(mlFrameTimes.back());
      mlbLost.push_back(mState == LOST);
    }
  }
  bool DefKLTTracking::LocalisationAndMapping()
  {
    bool bOK = TrackWithMotionModel();
    return bOK;
  }

  bool DefKLTTracking::DebugNeedNewKeyFrame()
  {
    std::cout << "id del frame : " << mCurrentFrame->mnId << std::endl;
    if (mCurrentFrame->mnId % 25 == 0)
    {
      newReferenceKeyframe_ = true;
      return true;
    }
    else if (mCurrentFrame->mnId % 5 == 0)
    {
      newReferenceKeyframe_ = false;
      return true;
    }
    else
    {
      return false;
    }
  }

  bool DefKLTTracking::NeedNewKeyFrame()
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

    for (int i = 0; i < mCurrentFrame->N; i++)
    {
      {
        if (mCurrentFrame->mvpMapPoints[i] && !mCurrentFrame->mvbOutlier[i])
          nTrackedClose++;
        else
          nNonTrackedClose++;
      }
    }

    float outlierPerc = float(nTrackedClose) / float(nNonTrackedClose + nTrackedClose);

    // Nodes viewed
    int nodesViewed = 0;

    std::set<Node *> Nodes =
        static_cast<DefMap *>(mpMap)->GetTemplate()->getNodes();
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
    cout << "Reprojection Error: " << mCurrentFrame->repError << endl;
    cout << "Outlier perc: " << outlierPerc << " " << perctOutliers_ << endl;

    bool bNeedToInsertClose = (ratioViewed < 0.75); // 0.4
    // bNeedToInsertClose = true;GrabImageMonocularGT

    bool badTemplate = (mCurrentFrame->repError > 1);
    if (bNeedToInsertClose)
    {
      newReferenceKeyframe_ = true;
    }
    else
    {
      newReferenceKeyframe_ = false;
    }

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

  void DefKLTTracking::TrackDataset(
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
          static_cast<DefMap *>(mpMap)->GetTemplate())
          ->getFacetTexture(keyframe);
      mState = OK;
    }
    if (static_cast<DefMap *>(mpMap)->GetTemplate())
    {
      Optimizer::DefPoseOptimization(
          mCurrentFrame, mpMap, this->getRegLap(), this->getRegInex(),
          this->getRegTemp(), LocalZone);
    }
    else
    {
      ORB_SLAM2::Optimizer::poseOptimization(mCurrentFrame);
    }
    if (viewerOn)
    {
      mpMapDrawer->SetCurrentCameraPose(mCurrentFrame->mTcw);
      mpFrameDrawer->Update(this);
      mpMapDrawer->UpdatePoints(mCurrentFrame);
    }
  }

  // Relocate the camera and get old template if system is lost
  bool DefKLTTracking::relocalization()
  {
    // Extract ORB for computing BoW (we just extract ORB in KFs)
    // Compute Bag of Words Vector
    mCurrentFrame->extractORBToRelocate();

    cout << "Extracting BoW..." << endl;
    mCurrentFrame->ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for
    // relocalisation
    vector<KeyFrame *> vpCandidateKFs =
        mpKeyFrameDB->DetectRelocalizationCandidates(mCurrentFrame);

    if (vpCandidateKFs.empty())
      return false;
    const int nKFs = vpCandidateKFs.size();

    cout << "There are some candidates: ";
    for (int i = 0; i < nKFs; i++)
    {
      cout << vpCandidateKFs[i]->mnId << " ";
    }
    cout << endl;

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75, true);

    vector<ORB_SLAM2::PnPsolver *> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint *>> vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

    for (int i = 0; i < nKFs; i++)
    {
      KeyFrame *pKF = vpCandidateKFs[i];
      if (pKF->isBad())
      {
        vbDiscarded[i] = true;
        cout << "Discarded (bad KF): " << pKF->mnId << endl;
      }
      else
      {
        int nmatches =
            matcher.SearchByBoW(pKF, *mCurrentFrame, vvpMapPointMatches[i]);
        std::cout << "matches of kf: " << nmatches << std::endl;
        if (nmatches < 15)
        {
          vbDiscarded[i] = true;
          cout << "Discarded (bad BoW matching): " << pKF->mnId << endl;
          continue;
        }
        else
        {
          ORB_SLAM2::PnPsolver *pSolver =
              new ORB_SLAM2::PnPsolver(*mCurrentFrame, vvpMapPointMatches[i]);
          // Increase threshold to allow points with deformation
          pSolver->SetRansacParameters(0.99, 8, 300, 4, 0.5, 20); //5.991 last position
          vpPnPsolvers[i] = pSolver;
          cout << "Accepted: " << pKF->mnId << " / Matches: " << nmatches << endl;
          nCandidates++;
        }
      }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9, true);

    KeyFrame *pKFreloc;
    KeyFrame *pKFref;

    while (nCandidates > 0 && !bMatch)
    {
      for (int i = 0; i < nKFs; i++)
      {
        if (vbDiscarded[i])
          continue;

        // Perform 5 Ransac Iterations
        vector<bool> vbInliers;
        int nInliers;
        bool bNoMore;

        ORB_SLAM2::PnPsolver *pSolver = vpPnPsolvers[i];
        cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

        // If Ransac reachs max. iterations discard keyframe
        if (bNoMore)
        {
          vbDiscarded[i] = true;
          nCandidates--;
        }

        cout << "Keyframe: " << vpCandidateKFs[i]->mnId << " / Inliers: " << nInliers << endl;

        // If a Camera Pose is computed, optimize
        if (!Tcw.empty())
        {
          Tcw.copyTo(mCurrentFrame->mTcw);

          set<MapPoint *> sFound;

          const int np = vbInliers.size();

          for (int j = 0; j < np; j++)
          {
            if (vbInliers[j])
            {
              mCurrentFrame->mvpMapPoints[j] = vvpMapPointMatches[i][j];
              sFound.insert(vvpMapPointMatches[i][j]);
            }
            else
              mCurrentFrame->mvpMapPoints[j] = NULL;
          }

          // Assign temporal template
          pKFreloc = vpCandidateKFs[i];
          cout << "Relocalization Keyframe: " << pKFreloc->mnId << endl;
          pKFref = static_cast<DefKeyFrame *>(pKFreloc)->getReferenceKeyframe();
          cout << "Reference Keyframe: " << pKFref->mnId << endl;

          std::unique_lock<std::mutex> M(static_cast<DefMap *>(mpMap)->MutexUpdating);
          static_cast<DefMap *>(mpMap)->clearTemplate();

          //static_cast<DefKeyFrame *>(pKFreloc)->assignTemplate();
          static_cast<DefMap *>(mpMap)->createTemplate(pKFref);

          int nGood = Optimizer::DefPoseOptimization(
              mCurrentFrame, mpMap, this->getRegLap(), this->getRegInex(), 0,
              LocalZone);

          if (nGood < 10)
            continue;

          // If few inliers, search by projection in a coarse window and optimize again
          if (nGood < 20)
          {
            int nadditional = matcher2.SearchByProjection(
                *mCurrentFrame, vpCandidateKFs[i], sFound, 15, 100);

            if (nadditional + nGood >= 40)
            {

              cout << "1_DEFORMABLE OPT in reloc" << endl;
              nGood = Optimizer::DefPoseOptimization(
                  mCurrentFrame, mpMap, this->getRegLap(), this->getRegInex(), 0,
                  LocalZone);

              // If many inliers but still not enough, search by projection again in a narrower window
              // the camera has been already optimized with many points
              if (nGood > 40 && nGood < 50)
              {
                sFound.clear();
                for (int ip = 0; ip < mCurrentFrame->N; ip++)
                  if (mCurrentFrame->mvpMapPoints[ip])
                    sFound.insert(mCurrentFrame->mvpMapPoints[ip]);
                nadditional = matcher2.SearchByProjection(
                    *mCurrentFrame, vpCandidateKFs[i], sFound, 5, 64);
                //std::cout << nadditional << std::endl;

                // Final optimization
                if (nGood + nadditional >= 50)
                {
                  cout << "2_DEFORMABLE OPT in reloc" << endl;
                  Optimizer::DefPoseOptimization(
                      mCurrentFrame, mpMap, this->getRegLap(), this->getRegInex(), 0,
                      LocalZone);

                  for (int io = 0; io < mCurrentFrame->N; io++)
                    if (mCurrentFrame->mvbOutlier[io])
                      mCurrentFrame->mvpMapPoints[io] = NULL;
                }
              }
            }
          }
          else
          {
            // Deformable optimization must be performed in any case

            cout << "3_DEFORMABLE OPT in reloc" << endl;
            nGood = Optimizer::DefPoseOptimization(
                mCurrentFrame, mpMap, this->getRegLap(), this->getRegInex(), 0,
                LocalZone);
          }

          // If the pose is supported by enough inliers stop ransacs and continue
          if (nGood >= 10) //50
          {
            bMatch = true;
            cout << "Relocalization succeded with Keyframe: " << pKFreloc->mnId << endl;
            break;
          }
        }
      }
    }

    if (!bMatch)
    {
      mCurrentFrame->mTcw = cv::Mat();
      return false;
    }
    else
    {
      mnLastRelocFrameId = mCurrentFrame->mnId;

      // We need to retrieve old template from the accepted KF
      mpReferenceKF = pKFreloc;
      mCurrentFrame->mpReferenceKF = mpReferenceKF;

      cout << "Assigning new template from KeyFrame " << pKFreloc->mnId << endl;

      cv::Mat imreloc = pKFreloc->imGray.clone();
      cv::imshow("Reloc Keyframe", imreloc);

      cout << "Frame lost: " << mCurrentFrame->mnId << endl;
      cout << "KeyFrame reloc: " << pKFreloc->mnId << endl;

      // Update KLT stuff
      mpLastKeyFrame = pKFreloc;
      mvKLTKeys = mCurrentFrame->mvKeys;
      mvKLTMPs = mCurrentFrame->mvpMapPoints;
      mvKLTStatus.resize(mvKLTKeys.size(), false);

      for (int i = 0; i < mvKLTMPs.size(); i++)
      {
        MapPoint *pMP = mvKLTMPs[i];
        if (pMP)
          mvKLTStatus[i] = true;
      }

      mCurrentFrame->SetTrackedPoints(mvKLTKeys, mvKLTStatus, mvKLTMPs, vHessian_);

      mKLTtracker.SetReferenceImage(mCurrentFrame->ImGray, mvKLTKeys);

      return true;
    }
  }

  void DefKLTTracking::UpdateLastFrame()
  {
    // Update pose according to reference keyframe
    KeyFrame *pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr * pRef->GetPose());

    if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR ||
        !mbOnlyTracking)
      return;
  }

  void DefKLTTracking::UpdateLocalPoints()
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
        /* if (!static_cast<DefMapPoint *>(pMP)->getFacet())
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

  cv::Mat DefKLTTracking::GrabImageMonocularGT(const cv::Mat &imRectLeft,
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
    std::cout << mImGray.size() << std::endl;

    int action = (mState == eTrackingState::NO_IMAGES_YET || mState == eTrackingState::NOT_INITIALIZED) ? 0 : 2;

    cout << "STATUS: " << mState << endl;

    mCurrentFrame = new GroundTruthFrame(
        mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef,
        mbf, mThDepth, imRectLeft, imGrayRight, action, _mask);

    this->Track();

    if ((mState == eTrackingState::OK) && (saveResults))
    {
      float scale =
          static_cast<GroundTruthFrame *>(mCurrentFrame)->Estimate3DScale(mpMap);
      scalefile << mCurrentFrame->mTimeStamp << " " << scale << std::endl;
      double error = static_cast<GroundTruthFrame *>(mCurrentFrame)
                         ->Estimate3DError(mpMap, scale);

      if (viewerOn)
      {
        mpMapDrawer->UpdatePoints(mCurrentFrame, scale);
        this->mpFrameDrawer->SetError(error / 1000);
      }
    }

    return mCurrentFrame->mTcw.clone();
  }

  cv::Mat DefKLTTracking::GrabImageMonocularCTGT(const cv::Mat &imRectLeft,
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
    mCurrentFrame = new GroundTruthFrame(
        mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef,
        mbf, mThDepth, imRectLeft, imDepth, true, _mask);

    this->Track();

    if ((mState == eTrackingState::OK) && (saveResults))
    {
      float scale =
          static_cast<GroundTruthFrame *>(mCurrentFrame)->Estimate3DScale(mpMap);
      scalefile << mCurrentFrame->mTimeStamp << " " << scale << std::endl;
      double error = static_cast<GroundTruthFrame *>(mCurrentFrame)
                         ->Estimate3DError(mpMap, scale);

      if (viewerOn)
      {
        mpMapDrawer->UpdatePoints(mCurrentFrame, scale);
        this->mpFrameDrawer->SetError(error);
      }
      // GroundTruthCalculator::CompareDeformationGT(&mCurrentFrame,timestamp);
    }
    return mCurrentFrame->mTcw.clone();
  }

  void DefKLTTracking::MonocularInitialization()
  {
    // Initialize the surface and the points in the surface considering a plane
    // parallel to the camera plane
    if (mCurrentFrame->N > 100)
    {
      // Set Frame pose to the origin
      mCurrentFrame->SetPose(cv::Mat::eye(4, 4, CV_32F));

      // Create KeyFrame
      KeyFrame *pKFini =
          new GroundTruthKeyFrame(*mCurrentFrame, mpMap, mpKeyFrameDB);

      cv::buildOpticalFlowPyramid(mCurrentFrame->ImGray, pKFini->imPyr, mKLTtracker.winSize, mKLTtracker.maxLevel);

      // Insert KeyFrame in the map
      mpMap->AddKeyFrame(pKFini);
      // Create MapPoints with inliers and associate to KeyFrame
      for (size_t i = 0; i < size_t(mCurrentFrame->N); i++)
      {
        cv::KeyPoint kp = mCurrentFrame->mvKeysUn[i];

        cv::Mat x3D = (cv::Mat_<float>(3, 1)
                           << (kp.pt.x - mCurrentFrame->cx) / mCurrentFrame->fx,
                       (kp.pt.y - mCurrentFrame->cy) / mCurrentFrame->fy, 1);
        //MapPoint *pNewMP = new DefMapPoint(x3D, pKFini, mpMap);
        cv::Vec3f normal;
        normal(0) = 0;
        normal(1) = 0;
        normal(2) = -1;

        MapPoint *pNewMP = new DefMapPoint(x3D, pKFini, mpMap, mCurrentFrame->mvKeys[i].pt, normal);

        pNewMP->trackedByKLT = true;
        pNewMP->AddObservation(pKFini, i);
        pKFini->addMapPoint(pNewMP, i);
        pNewMP->ComputeDistinctiveDescriptors();
        pNewMP->UpdateNormalAndDepth();
        mpMap->addMapPoint(pNewMP);
        mCurrentFrame->mvpMapPoints[i] = pNewMP;
        pKFini->GetMapPoint(i);
      }

      mvKLTKeys = mCurrentFrame->mvKeys;
      mvKLTMPs = mCurrentFrame->mvpMapPoints;
      mvKLTStatus.resize(mCurrentFrame->N, true);

      mKLTtracker.SetReferenceImage(mCurrentFrame->ImGray, mvKLTKeys);

      double *Array;
      Array = new double[_NumberOfControlPointsU * _NumberOfControlPointsV];

      for (uint i(0); i < _NumberOfControlPointsU * _NumberOfControlPointsV;
           i++)
      {
        Array[i] = 1;
      }
      BBS::bbs_t bbs;
      auto defkf = static_cast<DefKeyFrame *>(pKFini);
      bbs.umin = defkf->umin;
      bbs.umax = defkf->umax;
      bbs.nptsu = _NumberOfControlPointsU;
      bbs.nptsv = _NumberOfControlPointsV;
      bbs.vmax = defkf->vmax;
      bbs.vmin = defkf->vmin;
      bbs.valdim = 1;
      defkf->surface->saveArray(Array, bbs);

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
      static_cast<DefKeyFrame *>(pKFini)->assignTemplate();
      static_cast<DefMap *>(mpMap)->createInitialTemplate(pKFini);
      std::cout << static_cast<DefMap *>(mpMap)->GetTemplate()->getNodes().size() << std::endl;

      if (viewerOn)
      {
        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame->mTcw);
        mpFrameDrawer->Update(this);
        mpMapDrawer->UpdatePoints(mCurrentFrame);
        static_cast<DefMapDrawer *>(mpMapDrawer)->updateTemplate();
        static_cast<DefMapDrawer *>(mpMapDrawer)->updateTemplateAtRest();
      }
      mState = OK;
    }
  }

  void DefKLTTracking::CleanMatches()
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

  void DefKLTTracking::EraseTemporalPoints()
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

  bool DefKLTTracking::TrackWithMotionModel()
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
        if (!static_cast<DefMapPoint *>(pMP)->getFacet())
          mvKLTStatus[i] = false;
        if (mvKLTStatus[i])
          toTrack++;
      }
    }

    int nmatches = mKLTtracker.PRE_Track(mCurrentFrame->ImGray, mvKLTKeys, mvKLTStatus, vHessian_, true, 0.85);

    for (size_t i = 0; i < mvKLTMPs.size(); i++)
    {
      if (!mvKLTStatus[i])
        continue;
      if (mCurrentFrame->_mask.at<uchar>(mvKLTKeys[i].pt.y, mvKLTKeys[i].pt.x) < 125)
        mvKLTStatus[i] = false;
    }

    cout << "[TrackWithMotionModel]: points tracked by KLT: " << nmatches << " of " << toTrack << endl;

    mCurrentFrame->SetTrackedPoints(mvKLTKeys, mvKLTStatus, mvKLTMPs, vHessian_);

    if (debugPoints)
      printCurrentPoints("1_TrackingMotionModel");

    std::set<MapPoint *> setM;

    for (MapPoint *pMP : mCurrentFrame->mvpMapPoints)
    {
      setM.insert(pMP);
    }
    if (static_cast<DefMap *>(mpMap)->GetTemplate())
    {
      Optimizer::DefPoseOptimization(
          mCurrentFrame, mpMap, this->getRegLap(), this->getRegInex(),
          this->getRegTemp(), LocalZone);
    }
    //Optimizer::poseOptimization(mCurrentFrame);
    if (debugPoints)
      printCurrentPoints("2_OptimizationMotionModel");

    cout << "[TrackWithMotionModel]: " << mCurrentFrame->mvpMapPoints.size() << " --- " << setM.size() << endl;

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

    if (nmatches < 30)
      return false;

    return true;
  }

  void DefKLTTracking::UpdateSeeds()
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

  void DefKLTTracking::CreateNewKeyFrame()
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
    mvKLTStatus.resize(mvKLTKeys.size(), true);

    for (size_t i = 0; i < mvKLTMPs.size(); i++)
    {
      mvKLTStatus[i] = true;
      mvKLTMPs[i]->trackedByKLT = true;
    }

    //Set new reference image for KLT
    //mKLTtracker.SetReferenceImage(mCurrentFrame->ImGray, mvKLTKeys);
    mKLTtracker.SetReferenceImage(mCurrentFrame->ImGray, mvKLTMPs, mvKLTKeys);

    //Create new KeyFrame
    KeyFrame *pKF = new GroundTruthKeyFrame(*mCurrentFrame, mpMap, mpKeyFrameDB);
    if (newReferenceKeyframe_)
    {
      static_cast<DefKeyFrame *>(pKF)->kindKeyframe = DefKeyFrame::kindofKeyFrame::REFERENCE;
      std::cout << "KEYFRAME FOR REFERENCE" << std::endl;
    }
    else
    {
      static_cast<DefKeyFrame *>(pKF)->kindKeyframe = DefKeyFrame::kindofKeyFrame::REFINEMENT;
      std::cout << "KEYFRAME FOR REFINEMENT" << std::endl;
    }

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

  bool DefKLTTracking::TrackLocalMap()
  {
    // We have an estimation of the camera pose and some map points tracked in the
    // frame. We retrieve the local map and try to find matches to points in the
    // local map.
    UpdateLocalMap();
    int basePoints = SearchLocalMapPoints();
    //int basePoints = mCurrentFrame->N;

    if (debugPoints)
      printCurrentPoints("3_SearchKLTLocalMap");

    if (static_cast<DefMap *>(mpMap)->GetTemplate())
    {
      Optimizer::DefPoseOptimization(
          mCurrentFrame, mpMap, this->getRegLap(), this->getRegInex(),
          this->getRegTemp(), LocalZone);
    }
    else
    {
      ORB_SLAM2::Optimizer::poseOptimization(mCurrentFrame);
    }

    if (debugPoints)
      printCurrentPoints("4_OptimizationKLTLocalMap");
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
              if (static_cast<DefMapPoint *>(
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
    set<MapPoint *> refPts;
    for (auto pMP : points)
    {
      if (pMP)
      {
        if (pMP->isBad())
          continue;
        refPts.insert(pMP);
        //if (static_cast<DefMapPoint *>(pMP)->getFacet())
        if (mCurrentFrame->isInFrustum(pMP, 0.5))
        {
          numberLocalMapPoints++;
        }
      }
    }

    std::fstream myfile("matches.txt", std::ios::in | std::ios::out | std::ios::ate);
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(5)
        << uint(mCurrentFrame->mTimeStamp);
    myfile << out.str() << " " << numberLocalMapPoints << " " << mnMatchesInliers << " " << mnMatchesOutliers << std::endl;
    perctOutliers_ = float(mnMatchesInliers) / float(mnMatchesInliers + mnMatchesOutliers);
    myfile.close();

    vector<cv::KeyPoint> vAllGoodKeys, vNewKLTKeys;
    vector<MapPoint *> vAllGoodMps, vNewKLTMPs;
    vector<bool> vGood;
    vector<cv::Mat> vHessian;
    for (size_t i = 0; i < mCurrentFrame->N; i++)
    {
      if (mCurrentFrame->mvpMapPoints[i])
      {
        if (!mCurrentFrame->mvbOutlier[i] && mCurrentFrame->isInFrustum(mCurrentFrame->mvpMapPoints[i], 0.5))
        {
          vAllGoodKeys.push_back(mCurrentFrame->mvKeys[i]);
          vAllGoodMps.push_back(mCurrentFrame->mvpMapPoints[i]);
          vGood.push_back(true);
          vHessian.push_back(mCurrentFrame->vHessian_[i]);

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
    mCurrentFrame->SetTrackedPoints(vAllGoodKeys, vGood, vAllGoodMps, vHessian);

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

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if (mCurrentFrame->mnId < mnLastRelocFrameId + mMaxFrames &&
        mnMatchesInliers < 80)
      return false;

    if (mnMatchesInliers < 70)
      return false;
    else
      return true;
  }

  int DefKLTTracking::SearchLocalMapPoints()
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

        // Compute Homography
        // -Get Rfk and tfk
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
      vector<cv::Mat> vHessian;
      LucasKanadeTracker kltLocalTracker = LucasKanadeTracker(cv::Size(11, 11), 1, 5, 0.1, 1e-4);
      //Use KLT to estimate Projected Local MapPoints
      int goodKLT = kltLocalTracker.TrackWithInfoWithHH(mCurrentFrame->ImGray, vNextPts, vPrevPts, bStatus, vPatches, vGrad, vMean, vMean2, vH, 0.75, vHessian);

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
      mCurrentFrame->AppendTrackedPoints(vNextPts, bStatus, vMPs, vHessian);
    }

    return toReturn;
  }

  void DefKLTTracking::printCurrentPoints(string nameWindow)
  {
    vector<cv::KeyPoint> vCurrKeys = mCurrentFrame->mvKeys;
    int numberKeys = vCurrKeys.size();
    cv::Mat mImOutlier = mImRGB.clone();

    cv::namedWindow(nameWindow);

    cout << "Updating outliers..." << endl;
    cout << numberKeys << " keypoints" << endl;
    cout << "size of rematched: " << mCurrentFrame->vRematched_.size() << endl;

    const float r = 5;

    for (int i = 0; i < numberKeys; i++)
    {
      MapPoint *pMP = mCurrentFrame->mvpMapPoints[i];
      if (pMP)
      {
        cv::Point2f pt1, pt2;
        pt1.x = vCurrKeys[i].pt.x - r;
        pt1.y = vCurrKeys[i].pt.y - r;
        pt2.x = vCurrKeys[i].pt.x + r;
        pt2.y = vCurrKeys[i].pt.y + r;

        cv::KeyPoint kp = mCurrentFrame->ProjectPoints(pMP->GetWorldPos());

        if (mCurrentFrame->mvbOutlier[i]) // Outlier
        {
          cv::rectangle(mImOutlier, pt1, pt2, cv::Scalar(0, 0, 255));
          cv::circle(mImOutlier, vCurrKeys[i].pt, 2, cv::Scalar(0, 0, 255), -1);
          cv::line(mImOutlier, vCurrKeys[i].pt, kp.pt, cv::Scalar(0, 0, 255));
          cv::circle(mImOutlier, kp.pt, 2, cv::Scalar(240, 255, 255), -1);
        }
        else if (mCurrentFrame->vRematched_[i]) // Point rematched from map
        {
          cv::rectangle(mImOutlier, pt1, pt2, cv::Scalar(255, 255, 0));
          cv::circle(mImOutlier, vCurrKeys[i].pt, 2, cv::Scalar(255, 255, 0), -1);
          cv::line(mImOutlier, vCurrKeys[i].pt, kp.pt, cv::Scalar(255, 255, 0));
          cv::circle(mImOutlier, kp.pt, 2, cv::Scalar(255, 255, 0), -1);
          mCurrentFrame->vRematched_[i] = false;
        }
        else // Tracked inlier
        {
          cv::rectangle(mImOutlier, pt1, pt2, cv::Scalar(0, 255, 0));
          cv::circle(mImOutlier, vCurrKeys[i].pt, 2, cv::Scalar(0, 255, 0), -1);
          cv::line(mImOutlier, vCurrKeys[i].pt, kp.pt, cv::Scalar(0, 255, 0));
          cv::circle(mImOutlier, kp.pt, 2, cv::Scalar(240, 255, 255), -1);
        }
      }
    }

    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(5)
        << uint(mCurrentFrame->mTimeStamp);
    cv::imwrite(nameWindow + "-" + out.str() + ".png", mImOutlier);

    cv::imshow(nameWindow, mImOutlier);
  }

  void DefKLTTracking::printPointsWatchedByKeyframes(string nameWindow)
  {
    vector<cv::KeyPoint> vCurrKeys = mCurrentFrame->mvKeys;
    int numberKeys = vCurrKeys.size();
    cv::Mat mImColors = mImRGB.clone();

    cv::namedWindow(nameWindow);

    for (int i = 0; i < numberKeys; i++)
    {
      MapPoint *pMP = mCurrentFrame->mvpMapPoints[i];
      if (pMP)
      {
        if (!mCurrentFrame->mvbOutlier[i])
        {
          cv::KeyPoint kp = vCurrKeys[i];

          int observations = pMP->Observations();
          int maxObs = 5;
          double alpha = std::min(1.0, 1.0 * observations / maxObs);
          // Color range from blue (0 obs) to red (+10 obs)
          cv::circle(mImColors, kp.pt, 4, cv::Scalar(255 * (1 - alpha), 0, 255 * alpha), -1);
        }
      }
    }

    cv::imshow(nameWindow, mImColors);
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(5)
        << uint(mCurrentFrame->mTimeStamp);
    cv::imwrite(nameWindow + "-" + out.str() + ".png", mImColors);
  }
} // namespace defSLAM
