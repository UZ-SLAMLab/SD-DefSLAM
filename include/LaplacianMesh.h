
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

#ifndef LAPLACIANMESH_H
#define LAPLACIANMESH_H

#include "KeyFrame.h"
#include "MapPoint.h"
#include "TriangularMesh.h"
#include "set"

namespace Eigen {
typedef Matrix<double, 1, 1> Vector1d;
}

namespace ORB_SLAM2 {
class TriangularMesh;
class MapPoint;
class KeyFrame;
}
namespace defSLAM {
using ORB_SLAM2::MapPoint;
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::Map;

class LaplacianMesh : public TriangularMesh {
public:
  LaplacianMesh(std::set<MapPoint *> &mspMapPoints, Map *map);

  LaplacianMesh(std::vector<std::vector<double>> &vertex,
                std::vector<std::vector<int>> &index, Map *map);

  LaplacianMesh(std::set<MapPoint *> &mspMapPoints, Map *map, KeyFrame *kF);

  virtual ~LaplacianMesh();
  // Initialization
  void ExtractMeanCurvatures();

  const Eigen::Vector3d GetLaplacianCoord(Node *);

  Eigen::Vector1d const GetMeanCurvatureInitial(Node *n);

  const Eigen::Vector1d GetMeanCurvature(Node *n);

  Eigen::Vector3d const GetMeanCurvatureVector(Node *n);

  std::map<Node *, Eigen::Vector3d, std::less<Node *>,
           Eigen::aligned_allocator<std::pair<const Node *, Eigen::Vector3d>>>
      LaplacianCoords;
  double maxCurv;
};
}
#endif
