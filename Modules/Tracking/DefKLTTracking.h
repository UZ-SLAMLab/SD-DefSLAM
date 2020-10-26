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

#ifndef DEFKLTTRACKING_H
#define DEFKLTTRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <Tracking.h>
#include <mutex>
#include <opencv2/core/eigen.hpp>

#include "LucasKanadeTracker.h"
#include "SettingsLoader.h"
namespace ORB_SLAM2
{
  class Tracking;
  class ORBmatcher;
} // namespace ORB_SLAM2

namespace defSLAM
{
  using ORB_SLAM2::FrameDrawer;
  using ORB_SLAM2::KeyFrameDatabase;
  using ORB_SLAM2::Map;
  using ORB_SLAM2::MapDrawer;
  using ORB_SLAM2::ORBVocabulary;
  using ORB_SLAM2::System;
  using ORB_SLAM2::Tracking;

  class DeformationKeyFrame;

  /*
   * This class encapsulates camera tracking logic. It performs short-term data association using Lucas-Kanade
   * optical flow algorithm to track FAST corners that will be feed to a deformable pose optimization.
   */
  class DefKLTTracking : public Tracking
  {

  public:
    DefKLTTracking(System *pSys, ORBVocabulary *pVoc,
                   FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
                   Map *pMap, KeyFrameDatabase *pKFDB,
                   const string &strSettingPath,
                   const int sensor = ORB_SLAM2::System::MONOCULAR,
                   bool viewerOn = false);

    DefKLTTracking(System *pSys, ORBVocabulary *pVoc,
                   FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
                   Map *pMap, KeyFrameDatabase *pKFDB,
                   const SettingsLoader &strSettingPath,
                   const int sensor = ORB_SLAM2::System::MONOCULAR,
                   bool viewerOn = false);

  public:

    // First optimization step. It computes some matches from previous frame and estimates a coarse estimation of
    // the camera pose with a deformable pose optimization
    virtual bool TrackWithMotionModel();

    // Second optimization step. It reuses the local map to get new putative matches that will be fed to a
    // deformable pose optimzation
    virtual bool TrackLocalMap();

    // Rigid relocalization: retrieve pose with PnP and get template
    // from the keyframe used for relocalization
    virtual bool relocalization();

    virtual cv::Mat GrabImageMonocularGT(const cv::Mat &im,
                                         const cv::Mat &imRight,
                                         const double &timestamp,
                                         cv::Mat _mask = cv::Mat());

    virtual cv::Mat GrabImageMonocularCTGT(const cv::Mat &im,
                                           const cv::Mat &imDepth,
                                           const double &timestamp,
                                           cv::Mat _mask = cv::Mat());

    virtual void TrackDataset(const cv::Mat &im,
                              std::vector<std::vector<double>> &matches,
                              double timestamp, const cv::Mat &Tcw0);

  protected:
    void Track() override;

    virtual void CleanMatches();

    void EraseTemporalPoints();

    // More sophisticated way of selecting keyframes (Author J.Morlana)
    bool NeedNewKeyFrame() override;
    bool DebugNeedNewKeyFrame();
    void CreateNewKeyFrame() override;

    void MonocularInitialization() override;

    void UpdateLocalPoints() override;

    bool LocalisationAndMapping() override;

    void UpdateLastFrame() override;

  protected:
    KeyFrame *keyframe;
    uint LocalZone;
    ofstream myfile;

    // Lucas-Kande tracker
    LucasKanadeTracker mKLTtracker;

    std::vector<cv::KeyPoint> mvKLTKeys;    //KeyPoints tracked by Lucas-Kanade
    std::vector<MapPoint *> mvKLTMPs;       //MapPoints associated to KeyPoints tracked
    std::vector<bool> mvKLTStatus;          //Flags from Lucas-Kanade to tell if a track is good or no
    std::vector<cv::Mat> vHessian_;         //Hessian from Lucas-Kanade estimations

    bool newReferenceKeyframe_;
    float perctOutliers_;

  protected:
    // [Not used in the paper version] Projects MapPoints in the current image using as pose the one predicted
    // by the motion model
    void UpdateSeeds();

    // Searches new MapPoints in the local map. It projects to the current image and matches them with Lucas-Kanade
    int SearchLocalMapPoints();

    ///------------------------------------
    ///          Debugging
    ///------------------------------------
    bool debugPoints;
    void printCurrentPoints(string nameWindow);
    void printPointsWatchedByKeyframes(string nameWindow);
  };

} // namespace defSLAM

#endif // DEFORMABLE TRACKING_H
