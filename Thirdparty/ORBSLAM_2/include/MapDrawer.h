/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
* of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

//#include "Tracking.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

  // class Tracking;
  class MapDrawer
  {
  public:
    MapDrawer(Map *pMap, const string &strSettingPath);

    Map *mpMap; // ptr to the map.

    virtual void DrawMapPoints();
    virtual void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
    void DrawError();
    virtual void reset();
    // It draw the GT points
    virtual void DrawGTPoints();
    void virtual UpdatePoints(Frame *pFrame);
    void virtual UpdatePoints(Frame *pFrame, float s);
    void virtual DrawPoints();

  protected:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    vector<double> PointsMap, PointsSeen, PointsMono, PointsGT, PointsLocal,
        PointsStereoFrame, PointsAtRest, PointsStereoAtRest;
    vector<double> PointsSeenar, PointsMonoar, PointsGTar;
    vector<double> PointsRef;

    std::mutex mPoints;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;
  };

} // namespace ORB_SLAM2

#endif // MAPDRAWER_H
