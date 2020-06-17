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
#ifndef DEFKEYFRAME_H
#define DEFKEYFRAME_H
#include "MapPoint.h"

#include "KeyFrame.h"
#include "Surface.h"

#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace ORB_SLAM2 {
class KeyFrame;
class Map;
class Frame;
class MapPoint;
class Node;
class Template;
class Facet;

}

namespace defSLAM {
class DiffProp;
class PolySolver;
class Surface;
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::Map;
using ORB_SLAM2::Frame;
using ORB_SLAM2::MapPoint;
using ORB_SLAM2::KeyFrameDatabase;

class DeformationKeyFrame : public KeyFrame {
public:
  DeformationKeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);

  ~DeformationKeyFrame();

  void NormaliseKeypoints();

  double umin;
  double umax;
  double vmin;
  double vmax;
  int NCu;
  int NCv;
  int valdim;
  std::vector<cv::KeyPoint> mpKeypointNorm;
  std::vector<bool> Outliers;
  int KeyframesRelated;
  Surface *surface;
  float accMean;
  void assignTemplate();
  bool templateAssigned();

private:
  std::mutex mutex;
  bool haveATemplate_;
};

} // namespace ORB_SLAM2

#endif // DEFORMATION MAPPOINT_H
