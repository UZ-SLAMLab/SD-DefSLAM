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

#ifndef SHAPEFROMNORMALS_H
#define SHAPEFROMNORMALS_H

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "KeyFrame.h"
#include "Surface.h"
#include "WarpDatabase.h"
#include <ceres/ceres.h>

namespace defSLAM {
class Surface;
using ORB_SLAM2::Map;
using ORB_SLAM2::KeyFrame;
struct SfNEigen {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::MatrixXd LinearSystem;
  Eigen::SparseMatrix<double> LinearSystemSparse;
  Eigen::MatrixXd M;
  Eigen::MatrixXd B;
  Eigen::MatrixXd X;
  Eigen::MatrixXd Solaux;
};

class ShapeFromNormals {
public:
  ShapeFromNormals(KeyFrame *refKf, WarpDatabase *SchwarpDB,
                   double bendingWeight);

  ~ShapeFromNormals() { delete sfnEigen_; }

  template <typename T>
  bool operator()(T const *const *parameters, T *residuals) const;

  void Run();

  virtual bool InitialSolution();
  virtual void PointstoProcess(std::vector<size_t> &Pointstoprocess);
  void getSolutions(std::map<KeyFrame *, Eigen::MatrixXd> &SurfacesK);

private:
  void ObtainM(BBS::bbs_t &bbs, KeyFrame *Refdefkf, Eigen::MatrixXd &M);

private:
  KeyFrame *refKf;
  WarpDatabase *SchwarpDB;
  double bendingWeight;
  std::map<KeyFrame *, Surface> Surfaces;

  std::vector<double> u_vector;
  std::vector<double> v_vector;
  std::vector<size_t> PointIndex;
  int nPoints_RefKf, nPoints, Keyframestreated;
  // Order of the Keyframes in the optimisation. NULL if not optimised
  std::vector<KeyFrame *> Kflist;
  bool *mbAbort;
  Map *pMap;

  std::vector<KeyFrame *> KFOptimised;
  int ControlPointscols;
  int Mrows;
  int Brows;
  int Grows;
  int FixedKFrows;
  uint BestKeyframe;
  double umin, umax, vmin, vmax;
  void doInit() { sfnEigen_ = new SfNEigen; }
  SfNEigen *sfnEigen_;
};

} // namespace defSLAM

#endif // LOCALMAPPING_H
