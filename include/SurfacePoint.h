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

#ifndef SURFACEPOINT_H
#define SURFACEPOINT_H

#include "Surface.h"
#include <opencv2/core/core.hpp>

namespace defSLAM {

class Surface;

class SurfacePoint {
public:
  SurfacePoint() = delete;

  SurfacePoint(Surface *surface);

  ~SurfacePoint();

  // bool operator ==(const SurfacePoint &b) const;

  void setNormal(cv::Vec3f &N);

  void setDepth(cv::Vec3f &N);

  void getOriginalDepth(cv::Vec3f &x3d);

  void setOriginalDepth(cv::Vec3f &x3d);

  bool getNormal(cv::Vec3f &N);

  void getDepth(cv::Vec3f &x3D);

  bool thereisNormal();

private:
  Surface *surface;

  bool NormalOn;
  cv::Vec3f Normal;
  cv::Vec3f x3D;
  cv::Vec3f x3Do;
};

} // namespace ORB_SLAM2

#endif // DEFORMATION MAPPOINT_H
