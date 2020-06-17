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

#ifndef SURFACE_H
#define SURFACE_H

#include "MapPoint.h"
#include "iostream"

#include "SurfacePoint.h"
#include "bbs.h"
#include <Eigen/Dense>

namespace defSLAM {

class SurfacePoint;
struct SurfaceInt {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::MatrixXd L;
  Eigen::MatrixXd C;
  Eigen::MatrixXd EpsilonLambda;
  Eigen::MatrixXd Initial;
  Eigen::MatrixXd ImageCoordinates;
};

class Surface {
public:
  Surface() = delete;

  Surface(uint NumberOfPoints);

  Surface(std::vector<Eigen::Vector2d> &, std::vector<Eigen::Vector3d> &,
          uint NumberOfPoints);

  ~Surface();

  void SaveArray(double *Array, BBS::bbs_t &bbss);

  bool EnoughNormals();

  uint GetNumberofNormals();

  void SetNormalSurfacePoint(size_t ind, cv::Vec3f &N);

  bool GetNormalSurfacePoint(size_t ind, cv::Vec3f &N);

  void Set3DSurfacePoint(size_t ind, cv::Vec3f &x3D);

  void Get3DSurfacePoint(size_t ind, cv::Vec3f &N);

  void applyScale(double s22);

  void getVertex(std::vector<cv::Mat> &NodesSurface, uint, uint);

  uint getNormalsLimit() const;

private:
  double *NodesDepth;

  void doInit() { surfEig = new SurfaceInt; }

  SurfaceInt *surfEig;

  std::vector<SurfacePoint> SurfacePoints;

  bool NoInit;

  uint NumberofNormals;

  const uint normalsLimit_;
  BBS::bbs_t bbs;
};
}

#endif
