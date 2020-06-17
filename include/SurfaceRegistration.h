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

#ifndef SURFACEREGISTRATION_H
#define SURFACEREGISTRATION_H

#include "KeyFrame.h"

namespace ORB_SLAM2 {
class KeyFrame;
}
namespace defSLAM {
using ORB_SLAM2::KeyFrame;

class SurfaceRegistration {
public:
  SurfaceRegistration() = delete;

  SurfaceRegistration(KeyFrame *Keyframe, double chiLimit_ = 0.07,
                      bool check_chi = true);

  ~SurfaceRegistration();

  bool Register();
  float ScaleMinMedian(std::vector<std::vector<float>> &PosMono,
                       std::vector<std::vector<float>> &PosRef);

private:
  KeyFrame *refKF;
  bool check_chi;
  double chiLimit_;
};

} // namespace ORB_SLAM2

#endif // DEFORMATION MAPPOINT_H
