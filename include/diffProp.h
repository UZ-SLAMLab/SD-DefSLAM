
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

#ifndef DIFFPROP_H
#define DIFFPROP_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include <mutex>

namespace defSLAM {
using ORB_SLAM2::KeyFrame;

class DiffProp {
  /// Class with the differential properties of the warp
  /// used in normal estimation.
public:
  DiffProp() = default;
  ~DiffProp() = default;

public:
  std::pair<KeyFrame *, KeyFrame *> KFToKF;
  // Indexes
  size_t idx1;
  size_t idx2;
  // Image Points correspondences
  float I1u;
  float I1v;
  float I2u;
  float I2v;
  // Jacobian Image Points
  float J12a;
  float J12b;
  float J12c;
  float J12d;
  float J21a;
  float J21b;
  float J21c;
  float J21d;
  // Hessians Image Points
  float H12uux;
  float H12uuy;
  float H12uvx;
  float H12uvy;
  float H12vvx;
  float H12vvy;
  // Initial Normal
  float k1;
  float k2;
  float k12;
  float k22;
  // Outlier flag
  bool outlier;
};

} // namespace ORB_SLAM

#endif
