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

#ifndef SCHWARP_H
#define SCHWARP_H

#include "bbs.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>

#include <bbs_MAC.h>
#include <ceres/sized_cost_function.h>

namespace Warps {
class Warp : public ceres::CostFunction {
public:
  /* Initialize Schwarp
   * KP1, KP2 keypoints in reference frame and in queried frame respectively
   *
     umin, umax [scalars]: limits of the domain along the first input
     dimension (u-axis). By convention, the letters (u,v) are used in the
     input space of the spline while the letters (x,y,...) are used for the
     output space.
     vmin, vmax [scalars]: same as umin and umax but for the second
     input dimension (v-axis).
     nptsu, nptsv [scalars]: size of the control points grid. The total
     number of control points is nptsu*nptsv. The total number of parameters
     is valdim*nptsu*nptsv.
     valdim [scalar]: dimension of the B-Spline values. "valdim=1"
     correspond to a monovalued spline such as a range surface. "valdim=2"
     corresponds to, for example, an image warp.
  */
  Warp(const std::vector<cv::KeyPoint> &KP1,
       const std::vector<cv::KeyPoint> &KP2, std::vector<float> &invSigmas,
       double umin, double umax, double vmin, double vmax, int NCu, int NCv,
       int valdim, double fx, double fy);

  // Initialize warp with the given matches and with minimal bending energy
  static void initialize(
      std::vector<cv::KeyPoint> &KP1, std::vector<cv::KeyPoint> &KP2,
      double err, double umin, double umax, double vmin, double vmax, int NCu,
      int NCv, int valdim,
      double (&x)[_NumberOfControlPointsU * _NumberOfControlPointsV * 2]);

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const;

  static std::vector<cv::KeyPoint> getEstimates(
      std::vector<cv::KeyPoint> &KP1, double umin, double umax, double vmin,
      double vmax, int NCu, int NCv, int valdim,
      double (&x)[_NumberOfControlPointsU * _NumberOfControlPointsV * 2],
      Eigen::MatrixXd &Val, uint a, uint b);

private:
  BBS::bbs_t bbs;
  std::vector<double> values;
  std::vector<double> Jdata;
  const std::vector<cv::KeyPoint> &KP1;
  const std::vector<cv::KeyPoint> &KP2;
  const std::vector<float> invSigmas;
  double fx, fy; // focal length
};

class Schwarzian : public ceres::CostFunction {
public:
  /* Initialize Schwarp
   * KP1, KP2 keypoints in reference frame and in queried frame respectively
   *
     umin, umax [scalars]: limits of the domain along the first input
     dimension (u-axis). By convention, the letters (u,v) are used in the
     input space of the spline while the letters (x,y,...) are used for the
     output space.
     vmin, vmax [scalars]: same as umin and umax but for the second
     input dimension (v-axis).
     nptsu, nptsv [scalars]: size of the control points grid. The total
     number of control points is nptsu*nptsv. The total number of parameters
     is valdim*nptsu*nptsv.
     valdim [scalar]: dimension of the B-Spline values. "valdim=1"
     correspond to a monovalued spline such as a range surface. "valdim=2"
     corresponds to, for example, an image warp.
  */
  Schwarzian(double err, double umin, double umax, double vmin, double vmax,
             int valdim);
  virtual ~Schwarzian() {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const;

private:
  BBS::bbs_t bbs;
  std::vector<double> values;
  double err;
  double A21v[4 * _NumberOfControlPointsU * _NumberOfControlPointsV *
              _NumberOfControlPointsU * _NumberOfControlPointsV * 3];
  double B21v[6 * _NumberOfControlPointsU * _NumberOfControlPointsV *
              _NumberOfControlPointsU * _NumberOfControlPointsV * 2];
};

class Schwarp {
public:
  Schwarp(std::vector<double> KP1, std::vector<double> KP2, double *x,
          double reg, double umin, double umax, double vmin, double vmax,
          int NCu, int NCv);

private:
  static void initializate(std::vector<cv::KeyPoint> &KP1,
                           std::vector<cv::KeyPoint> &KP2, double err,
                           double umin, double umax, double vmin, double vmax,
                           int NCu, int NCv, int valdim, double *x);
  void estimateSchwarp(std::vector<cv::KeyPoint> &KP1,
                       std::vector<cv::KeyPoint> &KP2, double err, double umin,
                       double umax, double vmin, double vmax, int NCu, int NCv,
                       int valdim, double *x);
};
}
#endif
