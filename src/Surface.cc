#include "Surface.h"

namespace BBS {
void EvalEigen(bbs_t *bbs, const double *ctrlpts, double *u, double *v,
               int nval, Eigen::MatrixXd &val, int du, int dv);
}

namespace defSLAM {
Surface::Surface(uint NumberOfPoints)
    : NumberofNormals(0), NodesDepth(nullptr),
      SurfacePoints(NumberOfPoints, SurfacePoint(this)), normalsLimit_(10) {
  SurfacePoints.resize(NumberOfPoints, SurfacePoint(this));
  NoInit = true;
}

Surface::Surface(std::vector<Eigen::Vector2d> &Points2d,
                 std::vector<Eigen::Vector3d> &Points3d, uint NumberOfPoints)
    : NumberofNormals(0), NodesDepth(nullptr),
      SurfacePoints(NumberOfPoints, SurfacePoint(this)), normalsLimit_(10) {
  /// Define parametrization from 2D to 3D in the image
  /// We use thin-plate spline (TPS)
  std::vector<Eigen::Vector2d>::iterator it2d = Points2d.begin();
  std::vector<Eigen::Vector3d>::iterator it3d = Points3d.begin();
  Eigen::MatrixXd TPS(Points2d.size(), 5);
  uint i(0);
  for (; it2d != Points2d.end(); it2d++) {
    TPS.row(i).leftCols<2>() = (*it2d).transpose();
    TPS.row(i).rightCols<3>() = (*it3d).transpose();
    it3d++;
    i++;
  }
  NoInit = false;
}

Surface::~Surface() {
  SurfacePoints.clear();
  if (NodesDepth)
    delete[] NodesDepth;
  delete surfEig;
}

void Surface::SaveArray(double *Array, BBS::bbs_t &bbss) {
  bbs = bbss;
  if (NodesDepth)
    delete[] NodesDepth;

  NodesDepth = new double[bbs.nptsu * bbs.nptsv];
  for (uint i(0); i < bbs.nptsu * bbs.nptsv; i++) {
    NodesDepth[i] = Array[i];
  }
}

void Surface::getVertex(std::vector<cv::Mat> &NodesSurface, uint xs, uint ys) {
  double arrayCU[xs * ys];
  double arrayCV[xs * ys];

  uint us(0);
  double t(0.03);
  double umaxtemp = bbs.umax; //-0.20;
  double umintemp = bbs.umin; //+0.15;
  double vmaxtemp = bbs.vmax; //-0.20;
  double vmintemp = bbs.vmin; //+0.25;
  for (uint x(0); x < xs; x++) {
    for (uint j(0); j < ys; j++) {
      arrayCU[us] =
          double((umaxtemp - umintemp - 2 * t) * x) / (xs - 1) + (umintemp + t);
      arrayCV[us] =
          double((vmaxtemp - vmintemp - 2 * t) * j) / (ys - 1) + (vmintemp + t);
      us++;
    }
  }

  Eigen::MatrixXd Val2(xs * ys, 1);
  BBS::EvalEigen(&bbs, static_cast<const double *>(NodesDepth), arrayCU,
                 arrayCV, xs * ys, Val2, 0, 0);
  NodesSurface.reserve(xs * ys);
  for (uint x(0); x < xs * ys; x++) {
    cv::Mat x3D(4, 1, CV_32F);
    x3D.at<float>(0, 0) = arrayCU[x] * Val2(x, 0);
    x3D.at<float>(1, 0) = arrayCV[x] * Val2(x, 0);
    x3D.at<float>(2, 0) = Val2(x, 0);
    x3D.at<float>(3, 0) = 1;
    NodesSurface.push_back(x3D);
  }
}

void Surface::SetNormalSurfacePoint(size_t ind, cv::Vec3f &N) {
  if (ind >= SurfacePoints.size()) {
    std::cout << "WHAT" << std::endl;
  }
  if (!SurfacePoints[ind].thereisNormal()) {
    NumberofNormals++;
  }

  SurfacePoints[ind].setNormal(N);
}

bool Surface::GetNormalSurfacePoint(size_t ind, cv::Vec3f &N) {
  return SurfacePoints[ind].getNormal(N);
}

bool Surface::EnoughNormals() {
  std::cout << "Number Of normals " << NumberofNormals << " " << this
            << std::endl;
  return (NumberofNormals >= normalsLimit_);
}
uint Surface::GetNumberofNormals() { return NumberofNormals; }

void Surface::Set3DSurfacePoint(size_t ind, cv::Vec3f &x3D) {
  SurfacePoints[ind].setDepth(x3D);
}
void Surface::Get3DSurfacePoint(size_t ind, cv::Vec3f &x3D) {
  SurfacePoints[ind].getDepth(x3D);
}

void Surface::applyScale(double s22) {
  for (uint i(0); i < SurfacePoints.size(); i++) {
    cv::Vec3f x3D;
    SurfacePoints[i].getDepth(x3D);
    cv::Vec3f x3c = s22 * x3D;
    SurfacePoints[i].setDepth(x3c);
  }
  // std::cout << "reading array" << NodesDepth << std::endl;

  for (uint i(0); i < bbs.nptsu * bbs.nptsv; i++) {
    NodesDepth[i] = s22 * NodesDepth[i];
  }
}
}
