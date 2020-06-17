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

#ifndef DEFORBMATCHER_H
#define DEFORBMATCHER_H
#pragma once
#include <ORBmatcher.h>
#include <bbs_MAC.h>
#include <vector>

namespace defSLAM {
using ORB_SLAM2::ORBmatcher;
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::Frame;

class DeformationORBmatcher : public ORBmatcher {
public:
  DeformationORBmatcher(float nnratio = 0.6, bool checkOri = true);
  // normalise the keypoints 1/K*(xy1)
  void FindbySchwarp(
      KeyFrame *Kf1, KeyFrame *Kf2,
      vector<pair<size_t, size_t>> &vMatchedIndices,
      double (&x)[_NumberOfControlPointsU * _NumberOfControlPointsV * 2],
      double reg);

  void CalculateInitialSchwarp(
      KeyFrame *Kf1i, KeyFrame *Kf2i,
      vector<pair<size_t, size_t>> &vMatchedIndices,
      double (&x)[_NumberOfControlPointsU * _NumberOfControlPointsV * 2],
      double reg);

  int searchBySchwarp(
      KeyFrame *pKF1, KeyFrame *pKF2,
      double (&x)[_NumberOfControlPointsU * _NumberOfControlPointsV * 2],
      std::vector<pair<size_t, size_t>> &vMatchedPairs);

  int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame,
                         const float th, const bool bMono);

protected:
  void ProjectwithSchwarp(
      const cv::KeyPoint &kp1, cv::KeyPoint &kp2,
      double (&x)[_NumberOfControlPointsU * _NumberOfControlPointsV * 2],
      const KeyFrame *pKF2, const KeyFrame *KF1);
};

} // namespace ORB_SLAM

#endif // ORBMATCHER_H
