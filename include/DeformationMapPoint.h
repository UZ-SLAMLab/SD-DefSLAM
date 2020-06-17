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

#ifndef DEFMAPPOINT_H
#define DEFMAPPOINT_H
#include "MapPoint.h"

#include "DeformationMap.h"

#include "Facet.h"
#include <mutex>
#include <opencv2/core/core.hpp>


namespace ORB_SLAM2 {
class KeyFrame;
class Map;
class Frame;
class MapPoint;
}

namespace defSLAM {
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::Map;
using ORB_SLAM2::Frame;
using ORB_SLAM2::MapPoint;

class DeformationMapPoint : public MapPoint {
public:
  DeformationMapPoint() = delete;

  DeformationMapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap);
    ///-----------------------------------------------
  DeformationMapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap,cv::Point2f obs,cv::Vec3f normal);
    ///-----------------------------------------------
  DeformationMapPoint(const cv::Mat &Pos, Map *pMap, Frame *pFrame,
                      const int &idxF);

  virtual ~DeformationMapPoint() = default;

  void AssignedTemplate(const Template *);
  void RemoveTemplate();
  void SetFacet(Facet *face);
  Facet *getFacet();
  bool CheckFacet(Facet *face);
  void SetBadFlag() override;
  void SetCoordinates(double, double, double);
  void SetRGB(double, double, double);
  void getRGB(double &r, double &g, double &b);
  bool CheckReliability(Facet *, double, double, double);
  void SetWorldPos(cv::Mat, KeyFrame *);
  double b1, b2, b3;
  vector<double> bary1, bary2, bary3;
  void RecalculatePosition();

  cv::Mat GetWorldPosAtRest();
  void ExtractPatch(cv::Mat &im, cv::Point2d &ptK);
  void Repose();
  bool Deformable;
  bool InsertInTheTemplate;
  void SetNoFacet();
  bool nofacet;
  bool thereisface();
  std::map<KeyFrame *, std::pair<double, double>> k1k2s;
  void AddPositioninK(KeyFrame *kf, cv::Mat &X);
  std::unordered_map<KeyFrame *, cv::Mat> PosesKeyframes;
  void UpdateNormalAndDepth() override;

protected:
  const Template *mpTemplate;
  std::vector<Node *> Nodes;
  Facet *facet;
  double RGB[3];

protected:
  std::mutex MutexFacet;
  std::mutex MutexNFace;
};

} // namespace ORB_SLAM2

#endif // DEFORMATION MAPPOINT_H
