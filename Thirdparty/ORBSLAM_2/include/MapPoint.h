/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
*of Zaragoza) && Shaifali Parashar, Adrien Bartoli (Université Clermont Auvergne)
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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"

#include <map>
#include <mutex>
#include <opencv2/core/core.hpp>
namespace ORB_SLAM2
{

  class KeyFrame;
  class Map;
  class Frame;

  class MapPoint
  {
  public:
    MapPoint() = default;
    MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap);
    MapPoint(const cv::Mat &Pos, Map *pMap, Frame *pFrame, const int &idxF);

    virtual ~MapPoint() = default;

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame *GetReferenceKeyFrame();

    std::map<KeyFrame *, size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame *pKF, size_t idx);
    void EraseObservation(KeyFrame *pKF);

    int GetIndexInKeyFrame(KeyFrame *pKF);
    bool IsInKeyFrame(KeyFrame *pKF);

    virtual void setBadFlag();
    bool isBad();

    void Replace(MapPoint *pMP);
    MapPoint *GetReplaced();

    void IncreaseVisible(int n = 1);
    void IncreaseFound(int n = 1);
    float GetFoundRatio();
    inline int GetFound() { return mnFound; }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame *pKF);
    int PredictScale(const float &currentDist, Frame *pF);

    bool RealPositionKnown;
    Map *mpMap;

    bool assigned;
    double covNorm[2 * 2];

  public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    bool lastincorporasion;
    bool embedded;
    bool tracked;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;

    static std::mutex mGlobalMutex;

  protected:
    // Position in absolute coordinates
    cv::Mat mWorldPos;
    cv::Mat x3D;

    // Keyframes observing the point and associated index in keyframe
    std::map<KeyFrame *, size_t> mObservations;

    // Mean viewing direction
    cv::Mat mNormalVector;

    // Best descriptor to fast matching
    cv::Mat mDescriptor;

    // Reference KeyFrame
    KeyFrame *mpRefKF;

    // Tracking counters
    int mnVisible;
    int mnFound;

    // Bad flag (we do not currently erase MapPoint from memory)
    bool mbBad;
    MapPoint *mpReplaced;

    // Scale invariance distances
    float mfMinDistance;
    float mfMaxDistance;

    std::mutex mMutexPos;
    std::mutex mMutexGTPos;
    std::mutex mMutexFeatures;
  };

} // namespace ORB_SLAM2

#endif // MAPPOINT_H
