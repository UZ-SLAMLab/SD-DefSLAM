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

#ifndef GTKEYFRAME_H
#define GTKEYFRAME_H

#include "DeformationKeyFrame.h"
#include "MapPoint.h"

#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace defSLAM {

class DeformationKeyFrame;

class GroundTruthKeyFrame : public DeformationKeyFrame {
public:
  GroundTruthKeyFrame() = delete;

  GroundTruthKeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);

  ~GroundTruthKeyFrame() = default;

  void Estimate3DLocalMap(float s);

  void Estimate3DLocalMapIso(Map *map, double s);

  float Estimate3DScale();

  std::vector<cv::Point3f> mvLocalMapPoints, mvStereoMapPoints;

private:
  cv::Mat imRight;

  bool StereoAvaliable;

  std::vector<std::vector<float>> getPosMono();
  std::vector<std::vector<float>> getPosStereo();

private:
  std::vector<std::vector<float>> posMono_;
  std::vector<std::vector<float>> posStereo_;
  const int tempx_;
  const int tempy_;
  const int margin_;
  const int searchx_;
  const int searchy_;
  const double threshold_;
  const double radious_;

private:
  std::mutex mutexPoints;
};

} // namespace ORB_SLAM2

#endif // DEFORMATION MAPPOINT_H
