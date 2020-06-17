#include "SurfacePoint.h"

namespace defSLAM {

SurfacePoint::SurfacePoint(Surface *surface)
    : surface(surface), NormalOn(false) {}

SurfacePoint::~SurfacePoint() { surface = nullptr; }

void SurfacePoint::setNormal(cv::Vec3f &N) {
  Normal = N;
  NormalOn = true;
}

void SurfacePoint::setDepth(cv::Vec3f &x3d) { x3D = x3d; }

void SurfacePoint::setOriginalDepth(cv::Vec3f &x3d) { x3Do = x3d; }

bool SurfacePoint::getNormal(cv::Vec3f &N) {
  if (NormalOn) {
    N = Normal;
    return true;
  } else {
    return false;
  }
}

void SurfacePoint::getDepth(cv::Vec3f &x3d) { x3d = x3D; }

void SurfacePoint::getOriginalDepth(cv::Vec3f &x3d) { x3d = x3D; }

bool SurfacePoint::thereisNormal() { return NormalOn; }
} // namespace ORB_SLAM2
