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

//#define PARALLEL
//#define ORBSLAM

#include "System.h"
#include "Converter.h"

#ifdef ORBSLAM
#include "FrameDrawer.h"
#include "LocalMapping.h"
#include "Map.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "Viewer.h"
#else
#include "DeformationFrameDrawer.h"
#include "DeformationLocalMapping.h"
#include "DeformationMap.h"
#include "DeformationMapDrawer.h"
#include "DeformationTracking.h"
#include "DeformationViewer.h"
#endif
#include <iomanip>
#include <pangolin/pangolin.h>
#include <thread>
#include <unistd.h>

namespace ORB_SLAM2
{
#ifndef ORBSLAM
  using defSLAM::DeformationFrameDrawer;
  using defSLAM::DeformationLocalMapping;
  using defSLAM::DeformationMapDrawer;
  using defSLAM::DeformationTracking;
#endif
  using defSLAM::DeformationMap;
  using defSLAM::DeformationMapPoint;

  System::System(const string &strVocFile, const string &strSettingsFile,
                 const eSensor sensor, const bool bUseViewer)
      : mSensor(sensor), mpViewer(static_cast<Viewer *>(nullptr)), mbReset(false),
        mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false),
        mpLoopCloser(NULL)
  {
    // Output welcome message
    cout << endl
         << "DefSLAM 2019-2020 José Lamarca, University of Zaragoza." << endl
         << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
         << "This is free software, and you are welcome to redistribute it"
         << endl
         << endl;

    cout << "Input sensor was set to: ";

    if (mSensor != MONOCULAR)
    {
      cout << "Error" << endl;
      exit(-1);
    }
    // Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
      cerr << "Failed to open settings file at: " << strSettingsFile << endl;
      exit(-1);
    }

    // Load ORB Vocabulary
    cout << endl
         << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if (!bVocLoad)
    {
      cerr << "Wrong path to vocabulary. " << endl;
      cerr << "Falied to open at: " << strVocFile << endl;
      exit(-1);
    }
    cout << "Vocabulary loaded!" << endl
         << endl;

    // Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

// Create the Map
#ifndef ORBSLAM
    mpMap = new DeformationMap();
#else
    mpMap = new Map();
#endif

    if (bUseViewer)
    {
// Create Drawers. These are used by the Viewer
#ifndef ORBSLAM
      mpFrameDrawer = new DeformationFrameDrawer(mpMap);
      mpMapDrawer = new DeformationMapDrawer(mpMap, strSettingsFile);
#else
      mpFrameDrawer = new FrameDrawer(mpMap);
      mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
#endif
    }
// Initialize the Tracking thread
//(it will live in the main thread of execution, the one that called this
// constructor)
#ifdef ORBSLAM
    mpTracker =
        new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer, mpMap,
                     mpKeyFrameDatabase, strSettingsFile, mSensor, bUseViewer);

    // Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mpMapDrawer, mSensor == MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

#else
    mpTracker = new DeformationTracking(this, mpVocabulary, mpFrameDrawer,
                                        mpMapDrawer, mpMap, mpKeyFrameDatabase,
                                        strSettingsFile, mSensor, bUseViewer);

    // Initialize the Local Mapping thread and launch
    mpLocalMapper = new defSLAM::DeformationLocalMapping(
        mpMap, mpMapDrawer, mSensor == MONOCULAR, strSettingsFile);
#ifdef PARALLEL
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);
#endif
#endif

// Initialize the Loop Closing thread and launch
#ifndef ORBSLAM
    mpLoopCloser =
        nullptr; // new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary,
    //      mSensor != MONOCULAR);
    mptLoopClosing =
        nullptr; // new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
#else

    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary,
                                   mSensor != MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
#endif

    // Initialize the Viewer thread and launch
    if (bUseViewer)
    {
#ifndef ORBSLAM
      mpViewer = new defSLAM::Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker,
                                     strSettingsFile);
#else
      mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker,
                            strSettingsFile);
#endif
      mptViewer = new std::thread(&Viewer::Run, mpViewer);
      mpTracker->SetViewer(mpViewer);
    }

    // Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpLocalMapper->SetTracker(mpTracker);

#ifdef ORBSLAM
    mpTracker->SetLoopClosing(mpLoopCloser);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);
#else
    mpTracker->SetLoopClosing(nullptr);
    mpLocalMapper->SetLoopCloser(nullptr);
#endif
  }

  cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp,
                                 const cv::Mat _mask)
  {

    cv::Mat Mask;
    if (_mask.empty())
    {
      Mask = cv::Mat(im.rows, im.cols, CV_8UC1, cv::Scalar(255));
    }
    else
    {
      Mask = _mask.clone();
    }

    if (mSensor != MONOCULAR)
    {
      cerr << "ERROR: you called TrackMonocular but input sensor was not set to "
              "Monocular."
           << endl;
      exit(-1);
    }

    // Check mode change
    {
      unique_lock<mutex> lock(mMutexMode);
      if (mbActivateLocalizationMode)
      {
        mpLocalMapper->RequestStop();

        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->isStopped())
        {
          usleep(1000);
        }

        mpTracker->InformOnlyTracking(true);
        mbActivateLocalizationMode = false;
      }
      if (mbDeactivateLocalizationMode)
      {
        mpTracker->InformOnlyTracking(false);
        mpLocalMapper->Release();
        mbDeactivateLocalizationMode = false;
      }
    }

    // Check reset
    {
      unique_lock<mutex> lock(mMutexReset);
      if (mbReset)
      {
        mpTracker->Reset();
        mbReset = false;
      }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);
    if (mpViewer)
      mpViewer->Updatetimestamp(timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame->mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame->mvKeysUn;
    return Tcw;
  }

  void System::Restart(uint localzone, uint propagationzone)
  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);
    static_cast<DeformationMap *>(mpMap)->GetTemplate()->restart();
    std::vector<MapPoint *> mPoints = mpMap->GetAllMapPoints();

    for (std::vector<MapPoint *>::iterator pMP = mPoints.begin();
         pMP != mPoints.end(); pMP++)
    {
      if (static_cast<DeformationMapPoint *>(*pMP)->getFacet())
        static_cast<DeformationMapPoint *>(*pMP)->RecalculatePosition();
    }
  }

  cv::Mat System::TrackMonocularGT(const cv::Mat &im, const cv::Mat &imRight,
                                   const double &timestamp, const cv::Mat _mask)
  {
    if (mSensor != MONOCULAR)
    {
      cerr << "ERROR: you called TrackMonocular but input sensor was not set to "
              "Monocular."
           << endl;
      exit(-1);
    }
    cv::Mat Mask;
    if (_mask.empty())
    {
      Mask = cv::Mat(im.rows, im.cols, CV_8UC1, cv::Scalar(255));
    }
    else
    {
      Mask = _mask.clone();
    }
    // Check mode change
    {
      unique_lock<mutex> lock(mMutexMode);
      if (mbActivateLocalizationMode)
      {
        mpLocalMapper->RequestStop();

        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->isStopped())
        {
          usleep(2000);
        }

        mpTracker->InformOnlyTracking(true);
        mbActivateLocalizationMode = false;
      }
      if (mbDeactivateLocalizationMode)
      {
        mpTracker->InformOnlyTracking(false);
        mpLocalMapper->Release();
        mbDeactivateLocalizationMode = false;
      }
    }

    // Check reset
    {
      unique_lock<mutex> lock(mMutexReset);
      if (mbReset)
      {
        mpTracker->Reset();
        mbReset = false;
      }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocularGT(im, imRight, timestamp, Mask);
#ifndef ORBSLAM
#ifndef PARALLEL
    if (mpTracker->mState == Tracking::eTrackingState::OK)
      static_cast<DeformationLocalMapping *>(mpLocalMapper)->insideTheLoop();
#endif
    if (mpViewer)
    {
      mpViewer->Updatetimestamp(timestamp);
      std::cout << "waiting " << std::endl;
      while (!mpViewer->go())
        usleep(3000);
    }
#endif

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame->mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame->mvKeysUn;
    return Tcw;
  }

  cv::Mat System::TrackMonocularCTGT(const cv::Mat &im, const cv::Mat &CTdepth,
                                     const double &timestamp,
                                     const cv::Mat _mask)
  {
    if (mSensor != MONOCULAR)
    {
      cerr << "ERROR: you called TrackMonocular but input sensor was not set to "
              "Monocular."
           << endl;
      exit(-1);
    }
    cv::Mat Mask;
    if (_mask.empty())
    {
      Mask = cv::Mat(im.rows, im.cols, CV_8UC1, cv::Scalar(255));
    }
    else
    {
      Mask = _mask.clone();
    }
    // Check mode change
    {
      unique_lock<mutex> lock(mMutexMode);
      if (mbActivateLocalizationMode)
      {
        mpLocalMapper->RequestStop();

        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->isStopped())
        {
          usleep(2000);
        }

        mpTracker->InformOnlyTracking(true);
        mbActivateLocalizationMode = false;
      }
      if (mbDeactivateLocalizationMode)
      {
        mpTracker->InformOnlyTracking(false);
        mpLocalMapper->Release();
        mbDeactivateLocalizationMode = false;
      }
    }

    // Check reset
    {
      unique_lock<mutex> lock(mMutexReset);
      if (mbReset)
      {
        mpTracker->Reset();
        mbReset = false;
      }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocularCTGT(im, CTdepth, timestamp, Mask);
#ifndef ORBSLAM
#ifndef PARALLEL
    if (mpTracker->mState == Tracking::eTrackingState::OK)
      static_cast<DeformationLocalMapping *>(mpLocalMapper)->insideTheLoop();
#endif
    if (mpViewer)
    {
      mpViewer->Updatetimestamp(timestamp);
      while (!mpViewer->go())
        usleep(3000);
    }
#endif
    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame->mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame->mvKeysUn;
    return Tcw;
  }
  cv::Mat System::TrackMonocular(const cv::Mat &im, const cv::Mat &imRight,
                                 const double &timestamp, const cv::Mat _mask)
  {

    if (mSensor != MONOCULAR)
    {
      cerr << "ERROR: you called TrackMonocular but input sensor was not set to "
              "Monocular."
           << endl;
      exit(-1);
    }

    // Check mode change
    {
      unique_lock<mutex> lock(mMutexMode);
      if (mbActivateLocalizationMode)
      {
        mpLocalMapper->RequestStop();

        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->isStopped())
        {
          usleep(2000);
        }

        mpTracker->InformOnlyTracking(true);
        mbActivateLocalizationMode = false;
      }
      if (mbDeactivateLocalizationMode)
      {
        mpTracker->InformOnlyTracking(false);
        mpLocalMapper->Release();
        mbDeactivateLocalizationMode = false;
      }
    }

    // Check reset
    {
      unique_lock<mutex> lock(mMutexReset);
      if (mbReset)
      {
        mpTracker->Reset();
        mbReset = false;
      }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im, imRight, timestamp);

    if (mpViewer)
      mpViewer->Updatetimestamp(timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame->mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame->mvKeysUn;
    return Tcw;
  }

  bool System::RelocateMonocular(const cv::Mat &im, const double &timestamp,
                                 const cv::Mat _mask)
  {

    if (mSensor != MONOCULAR)
    {
      cerr << "ERROR: you called TrackMonocular but input sensor was not set to "
              "Monocular."
           << endl;
      exit(-1);
    }

    // Check mode change
    {
      unique_lock<mutex> lock(mMutexMode);
      if (mbActivateLocalizationMode)
      {
        mpLocalMapper->RequestStop();

        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->isStopped())
        {
          usleep(1000);
        }

        mpTracker->InformOnlyTracking(true);
        mbActivateLocalizationMode = false;
      }
      if (mbDeactivateLocalizationMode)
      {
        mpTracker->InformOnlyTracking(false);
        mpLocalMapper->Release();
        mbDeactivateLocalizationMode = false;
      }
    }

    // Check reset
    {
      unique_lock<mutex> lock(mMutexReset);
      if (mbReset)
      {
        mpTracker->Reset();
        mbReset = false;
      }
    }

    bool bOK = mpTracker->RelocateImageMonocular(im, timestamp);
    mpViewer->Updatetimestamp(timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame->mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame->mvKeysUn;
    return bOK;
  }

  void System::ActivateLocalizationMode()
  {
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
  }

  void System::DeactivateLocalizationMode()
  {
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
  }

  bool System::MapChanged()
  {
    static int n = 0;
    int curn = mpMap->GetLastBigChangeIdx();
    if (n < curn)
    {
      n = curn;
      return true;
    }
    else
      return false;
  }

  void System::Reset()
  {
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
  }

  void System::Shutdown()
  {
    mpLocalMapper->RequestFinish();
    if (mpLoopCloser)
      mpLoopCloser->RequestFinish();
    if (mpViewer)
    {
      mpViewer->RequestFinish();
      while (!mpViewer->isFinished())
        usleep(5000);
    }

    // Wait until all thread have effectively stopped
    if (mpLoopCloser)
      while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() ||
             mpLoopCloser->isRunningGBA())
      {
        usleep(5000);
      }

    if (mpViewer)
#ifdef ORBSLAM
      pangolin::BindToContext("ORBSLAM2: Map Viewer");
#else
      pangolin::BindToContext("DefSLAM: Map Viewer");
#endif
  }

  void System::SaveTrajectoryTUM(const string &filename)
  {
    cout << endl
         << "Saving camera trajectory to " << filename << " ..." << endl;
    if (mSensor == MONOCULAR)
    {
      cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
      return;
    }

    vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized
    // by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative
    // transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
    // a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                 lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++, lbL++)
    {
      if (*lbL)
        continue;

      KeyFrame *pKF = *lRit;

      cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

      // If the reference keyframe was culled, traverse the spanning tree to get a
      // suitable keyframe.
      while (pKF->isBad())
      {
        Trw = Trw * pKF->mTcp;
        pKF = pKF->GetParent();
      }

      Trw = Trw * pKF->GetPose() * Two;

      cv::Mat Tcw = (*lit) * Trw;
      cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
      cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

      vector<float> q = Converter::toQuaternion(Rwc);

      f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0)
        << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0]
        << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl
         << "trajectory saved!" << endl;
  }

  void System::SaveKeyFrameTrajectoryTUM(const string &filename)
  {
    cout << endl
         << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    // cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
      KeyFrame *pKF = vpKFs[i];

      // pKF->SetPose(pKF->GetPose()*Two);

      if (pKF->isBad())
        continue;

      cv::Mat R = pKF->GetRotation().t();
      vector<float> q = Converter::toQuaternion(R);
      cv::Mat t = pKF->GetCameraCenter();
      f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " "
        << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " "
        << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }

    f.close();
    cout << endl
         << "trajectory saved!" << endl;
  }

  void System::SaveTrajectoryKITTI(const string &filename)
  {
    cout << endl
         << "Saving camera trajectory to " << filename << " ..." << endl;
    if (mSensor == MONOCULAR)
    {
      cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
      return;
    }

    vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized
    // by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative
    // transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
    // a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                 lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++)
    {
      ORB_SLAM2::KeyFrame *pKF = *lRit;

      cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

      while (pKF->isBad())
      {
        //  cout << "bad parent" << endl;
        Trw = Trw * pKF->mTcp;
        pKF = pKF->GetParent();
      }

      Trw = Trw * pKF->GetPose() * Two;

      cv::Mat Tcw = (*lit) * Trw;
      cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
      cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

      f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1)
        << " " << Rwc.at<float>(0, 2) << " " << twc.at<float>(0) << " "
        << Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " "
        << Rwc.at<float>(1, 2) << " " << twc.at<float>(1) << " "
        << Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " "
        << Rwc.at<float>(2, 2) << " " << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl
         << "trajectory saved!" << endl;
  }

  int System::GetTrackingState()
  {
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
  }
} // namespace ORB_SLAM2
