#include "ShapeFromNormals.h"
#include "bbs_coloc.h"
//#include "Schwarp.h"
#include "DeformationKeyFrame.h"
#include "LinearSystemForCeres.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Timer.h"
#include "bbs.h"
#include <Converter.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <ctime>

#include <Eigen/QR>
#include <Eigen/SVD>
#include <bbs_MAC.h>

namespace BBS {
void colocEigen(bbs_t *bbs, double *u, double *v, int nsites,
                Eigen::MatrixXd &colocMatrix);
void BendingEigen(bbs_t *bbs, double err,
                  Eigen::SparseMatrix<double> &benMatrix);
int coloc_derivEigen(bbs_t *bbs, double *u, double *v, int nsites, int du,
                     int dv, Eigen::MatrixXd &colocMatrix);
void EvalEigen(bbs_t *bbs, const double *ctrlpts, double *u, double *v,
               int nval, Eigen::MatrixXd &val, int du, int dv);
}

namespace defSLAM {
  
ShapeFromNormals::ShapeFromNormals(KeyFrame *refKf, WarpDatabase *SchwarpDB,
                                   double bendingWeight)
    : refKf(refKf), SchwarpDB(SchwarpDB), bendingWeight(bendingWeight) {
  DeformationKeyFrame *Refdefkf = static_cast<DeformationKeyFrame *>(refKf);
  this->doInit();
  this->umin = Refdefkf->umin;
  this->umax = Refdefkf->umax;
  this->vmin = Refdefkf->vmin;
  this->vmax = Refdefkf->vmax;

  BBS::bbs_t bbsM;
  bbsM.umin = this->umin;
  bbsM.umax = this->umax;
  bbsM.nptsu = _NumberOfControlPointsU;
  bbsM.nptsv = _NumberOfControlPointsV;
  bbsM.vmax = this->vmax;
  bbsM.vmin = this->vmin;
  bbsM.valdim = 1;

  std::vector<Eigen::MatrixXd> Ms;
  std::vector<Eigen::SparseMatrix<double>> Bs;
  this->KFOptimised.push_back(refKf);
  Eigen::MatrixXd Mr;
  this->ObtainM(bbsM, refKf, Mr);
  Ms.push_back(Mr);

  Eigen::SparseMatrix<double> smBendinga(
      (_NumberOfControlPointsU * _NumberOfControlPointsV),
      (_NumberOfControlPointsU * _NumberOfControlPointsV));
  BBS::BendingEigen(&bbsM, bendingWeight, smBendinga);
  Bs.push_back(smBendinga);
  this->nPoints_RefKf = Ms[0].rows();

  this->ControlPointscols = _NumberOfControlPointsU * _NumberOfControlPointsV;

  this->Mrows = 0;
  for (uint i(0); i < Ms.size(); i++) {
    this->Mrows = this->Mrows + Ms[i].rows();
  }

  this->Brows = this->KFOptimised.size() * _NumberOfControlPointsU *
                _NumberOfControlPointsV;

  this->FixedKFrows = _NumberOfControlPointsU * _NumberOfControlPointsV;

  sfnEigen_->B.resize(Ms[0].rows() + Bs[0].rows(), 1);
  sfnEigen_->B.setZero(Ms[0].rows() + Bs[0].rows(), 1);
  sfnEigen_->X.resize(ControlPointscols, 1);
  sfnEigen_->X.setZero(ControlPointscols, 1);
  sfnEigen_->LinearSystem.resize(Ms[0].rows() + Bs[0].rows(), Ms[0].cols());
  sfnEigen_->LinearSystem.block(0, 0, Ms[0].rows(), Ms[0].cols()) = Ms[0];
  sfnEigen_->LinearSystem.block(Ms[0].rows(), 0, Bs[0].rows(), Bs[0].cols()) =
      Eigen::MatrixXd(Bs[0]);

  //// Initial Solution
  uint us(0);
  Eigen::MatrixXd p_roi(_NumberOfControlPointsU * _NumberOfControlPointsV, 2);
  for (int x(0); x < _NumberOfControlPointsU; x++) {
    for (int j(0); j < _NumberOfControlPointsV; j++) {
      p_roi(us, 0) =
          double((umax - umin) * x) / (_NumberOfControlPointsU - 1) + umin;
      p_roi(us, 1) =
          double((vmax - vmin) * j) / (_NumberOfControlPointsV - 1) + vmin;
      us++;
    }
  }
}

bool ShapeFromNormals::InitialSolution() {
  int Mrowsaux = nPoints_RefKf;
  int Browsaux = _NumberOfControlPointsU * _NumberOfControlPointsV;

  Eigen::MatrixXd Bauxaux(Mrowsaux + Browsaux + 1, 1);
  double MeanDepth(static_cast<DeformationKeyFrame *>(this->refKf)
                       ->accMean); // Mandala experiments
  std::cout << "MEANN DEPTH : " << MeanDepth << std::endl;
  Bauxaux << sfnEigen_->B.block(0, 0, Mrowsaux + Browsaux, 1),
      Browsaux * MeanDepth;
  Eigen::MatrixXd LynearAuxaux(Mrowsaux + Browsaux + 1, Browsaux);
  LynearAuxaux << sfnEigen_->LinearSystem.block(0, 0, Mrowsaux + Browsaux,
                                                Browsaux),
      Eigen::MatrixXd::Ones(1, Browsaux);

  //  Eigen::MatrixXd Solauxs =
  //  LynearAuxaux.colPivHouseholderQr().solve(Bauxaux);
  Eigen::MatrixXd Solauxs = LynearAuxaux.householderQr().solve(Bauxaux);
  std::cout << Browsaux * MeanDepth << "  "
            << LynearAuxaux.block(Mrowsaux + Browsaux, 0, 1, Browsaux) * Solauxs
            << " " << Solauxs.mean() << std::endl;
  // std::cout << "Sol SfN norm "
  // <<(LynearAuxaux*Solaux-Bauxaux).norm()/std::sqrt(Bauxaux.rows())<< " " <<
  // std::sqrt(Bauxaux.rows()) << std::endl;
  DeformationKeyFrame *defrefKf =
      static_cast<DeformationKeyFrame *>(this->refKf);

  for (uint var = 0; var < this->refKf->mvKeysUn.size(); ++var) {
    u_vector.push_back(defrefKf->mpKeypointNorm[var].pt.x);
    v_vector.push_back(defrefKf->mpKeypointNorm[var].pt.y);
  }

  if (u_vector.size() == 0)
    return false;

  for (uint i(0); i < Solauxs.rows(); i++)
    if (std::isnan(Solauxs(i))) {
      std::cout << "nan fail" << std::endl;
      return false;
    }
  for (uint i(0); i < Solauxs.rows(); i++)
    if (std::isinf(Solauxs(i))) {
      std::cout << "inf fail" << std::endl;
      return false;
    }

  double *Array;
  Array = new double[_NumberOfControlPointsU * _NumberOfControlPointsV];

  std::vector<float> depthVec;
  depthVec.reserve(Solauxs.rows());
  for (uint i(0); i < _NumberOfControlPointsU * _NumberOfControlPointsV; i++) {
    depthVec.push_back(Solauxs(i));
  }
  std::sort(depthVec.begin(), depthVec.end());
  float corr = 1 / (depthVec[depthVec.size() / 2]);
  sfnEigen_->Solaux.resize(_NumberOfControlPointsU * _NumberOfControlPointsV,
                           1);
  for (uint i(0); i < _NumberOfControlPointsU * _NumberOfControlPointsV; i++) {
    Array[i] = corr * Solauxs(i);
    sfnEigen_->Solaux(i) = corr * Solauxs(i);
  }

  BBS::bbs_t bbs;
  bbs.umin = this->umin;
  bbs.umax = this->umax;
  bbs.nptsu = _NumberOfControlPointsU;
  bbs.nptsv = _NumberOfControlPointsV;
  bbs.vmax = this->vmax;
  bbs.vmin = this->vmin;
  bbs.valdim = 1;

  Eigen::MatrixXd Val(u_vector.size(), 1);

  BBS::EvalEigen(&bbs, static_cast<double *>(Array), &u_vector[0], &v_vector[0],
                 u_vector.size(), Val, 0, 0);

  for (uint i(0); i < Val.rows(); i++) {
    cv::Vec3f X3d;
    X3d(0) = u_vector[i] * Val(i, 0);
    X3d(1) = v_vector[i] * Val(i, 0);
    X3d(2) = Val(i, 0);
    defrefKf->surface->Set3DSurfacePoint(i, X3d);
  }

  defrefKf->surface->SaveArray(Array, bbs);

  delete[] Array;
  return true;
}

void ShapeFromNormals::getSolutions(
    std::map<KeyFrame *, Eigen::MatrixXd> &SurfacesK) {}

void ShapeFromNormals::PointstoProcess(std::vector<size_t> &Pointstoprocess) {
  Pointstoprocess.swap(PointIndex);
}

template <typename T>
bool ShapeFromNormals::operator()(T const *const *parameters,
                                  T *residuals) const {
  for (int k = 0; k < sfnEigen_->LinearSystem.rows(); k++) {
    residuals[k] = T(0.00000);
  }

  for (int k = 0; k < sfnEigen_->LinearSystemSparse.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(
             sfnEigen_->LinearSystemSparse, k);
         it; ++it) {
      residuals[it.row()] += it.value() * parameters[0][it.col()];
    }
  }

  return true;
}

void ShapeFromNormals::Run() {
  int Mrowsaux = nPoints_RefKf;
  int Browsaux = _NumberOfControlPointsU * _NumberOfControlPointsV;
  DeformationKeyFrame *defrefKf =
      static_cast<DeformationKeyFrame *>(this->refKf);

  Eigen::MatrixXd Bauxaux(Mrowsaux + Browsaux + 1, 1);
  double MeanDepth(static_cast<DeformationKeyFrame *>(this->refKf)
                       ->accMean); // Mandala experiments
  std::cout << "MEANN DEPTH : " << MeanDepth << std::endl;
  Bauxaux << sfnEigen_->B.block(0, 0, Mrowsaux + Browsaux, 1),
      Browsaux * MeanDepth;
  Eigen::MatrixXd LynearAuxaux(Mrowsaux + Browsaux + 1, Browsaux);
  LynearAuxaux << sfnEigen_->LinearSystem.block(0, 0, Mrowsaux + Browsaux,
                                                Browsaux),
      Eigen::MatrixXd::Ones(1, Browsaux);

  ceres::Problem problem;

  ceres::CostFunction *cost_function = new ceres::NumericDiffCostFunction<
      LinearSystemForCeres, ceres::CENTRAL, ceres::DYNAMIC,
      _NumberOfControlPointsU * _NumberOfControlPointsV>(
      new LinearSystemForCeres(LynearAuxaux, Bauxaux), ceres::TAKE_OWNERSHIP,
      LynearAuxaux.rows());

  sfnEigen_->LinearSystemSparse = LynearAuxaux.sparseView();

  double *x;
  x = new double[_NumberOfControlPointsU * _NumberOfControlPointsV];
  sfnEigen_->Solaux =
      Eigen::MatrixXd(_NumberOfControlPointsU * _NumberOfControlPointsV, 1);
  for (uint i(0); i < _NumberOfControlPointsU * _NumberOfControlPointsV; i++) {
    x[i] = sfnEigen_->Solaux(i, 0);
  }

  problem.AddParameterBlock(x,
                            _NumberOfControlPointsU * _NumberOfControlPointsV);
  problem.AddResidualBlock(cost_function, NULL, x);

  // Run the solver!
  ceres::Solver::Options options;
  options.dynamic_sparsity = true;
  options.num_threads = 1;
  // options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;

  Solve(options, &problem, &summary);
  if (summary.IsSolutionUsable()) {
    for (uint i(0); i < Kflist.size(); i++) {
      if (Kflist[i]) {
        Eigen::Map<Eigen::Matrix<
            double, _NumberOfControlPointsU * _NumberOfControlPointsV, 1>>
            Xoutput(x +
                    (i * _NumberOfControlPointsU * _NumberOfControlPointsV));
      }
    }
  }

  BBS::bbs_t bbs;
  bbs.umin = this->umin;
  bbs.umax = this->umax;
  bbs.nptsu = _NumberOfControlPointsU;
  bbs.nptsv = _NumberOfControlPointsV;
  bbs.vmax = this->vmax;
  bbs.vmin = this->vmin;
  bbs.valdim = 1;

  Eigen::MatrixXd Val(u_vector.size(), 1);

  BBS::EvalEigen(&bbs, static_cast<double *>(x), &u_vector[0], &v_vector[0],
                 u_vector.size(), Val, 0, 0);

  for (uint i(0); i < Val.rows(); i++) {
    cv::Vec3f X3d;
    X3d(0) = u_vector[i] * Val(i, 0);
    X3d(1) = v_vector[i] * Val(i, 0);
    X3d(2) = Val(i, 0);
    // std::cout <<"Depth : " << Val(i,0) << std::endl;
    defrefKf->surface->Set3DSurfacePoint(i, X3d);
  }

  defrefKf->surface->SaveArray(x, bbs);

  delete[] x;
}

void ShapeFromNormals::ObtainM(BBS::bbs_t &bbs, KeyFrame *Refkf,
                               Eigen::MatrixXd &M) {
  DeformationKeyFrame *Refdefkf = static_cast<DeformationKeyFrame *>(Refkf);
  Surface *sref = (Refdefkf->surface);
  std::vector<double> u_vector;
  std::vector<double> v_vector;

  std::vector<cv::Vec3f> Normals;
  u_vector.reserve(Refkf->mvKeysUn.size());
  v_vector.reserve(Refkf->mvKeysUn.size());
  Normals.reserve(Refkf->mvKeysUn.size());
  for (uint var = 0; var < Refkf->mvKeysUn.size(); ++var) {
    cv::Vec3f Normal;
    auto mp = Refkf->GetMapPoint(var);
    if (!mp)
      continue;
    if (mp->isBad())
      continue;

    if (sref->GetNormalSurfacePoint(var, Normal)) {
      if (Normal == Normal) {
        // if (!mp->covNorm)
        //  continue;
        Normals.push_back(Normal);
        u_vector.push_back(Refdefkf->mpKeypointNorm[var].pt.x);
        v_vector.push_back(Refdefkf->mpKeypointNorm[var].pt.y);
      }
    }
  }

  double u[u_vector.size()];
  std::copy(u_vector.begin(), u_vector.end(), u);
  double v[v_vector.size()];
  std::copy(v_vector.begin(), v_vector.end(), v);

  Eigen::MatrixXd coloc = Eigen::MatrixXd::Zero(
      u_vector.size(), _NumberOfControlPointsU * _NumberOfControlPointsV);
  BBS::colocEigen(&bbs, u, v, u_vector.size(), coloc);

  Eigen::MatrixXd coloc_du = Eigen::MatrixXd::Zero(
      u_vector.size(), _NumberOfControlPointsU * _NumberOfControlPointsV);
  Eigen::MatrixXd coloc_dv = Eigen::MatrixXd::Zero(
      u_vector.size(), _NumberOfControlPointsU * _NumberOfControlPointsV);

  BBS::coloc_derivEigen(&bbs, u, v, u_vector.size(), 1, 0, coloc_du);
  BBS::coloc_derivEigen(&bbs, u, v, u_vector.size(), 0, 1, coloc_dv);

  uint npts(u_vector.size());
  uint nparams(_NumberOfControlPointsU * _NumberOfControlPointsV);
  Eigen::MatrixXd Mdense = Eigen::MatrixXd::Zero(2 * npts, nparams);
  M.resize(2 * npts, nparams);

  for (uint i(0); i < npts; i++) {
    Eigen::Vector3d n;
    n << Normals[i](0), Normals[i](1), Normals[i](2);
    n.normalize();
    Eigen::Vector3d etat, etatu, etatv;
    etat << u[i], v[i], 1;
    etatu << 1, 0, 0;
    etatv << 0, 1, 0;
    //  std::cout << "Normal : " <<n << std::endl;
    Eigen::MatrixXd sau(3, coloc_du.cols());
    sau << etat * coloc_du.row(i);

    Eigen::MatrixXd sa2(3, coloc_du.cols());
    sa2 << etatu * coloc.row(i);
    // std::cout << "sa2" << sa2 << std::endl;

    Eigen::MatrixXd Mi(2, coloc_du.cols());
    Mi << n.transpose() * (sau + sa2),
        (n.transpose() * (etat * coloc_dv.row(i) + etatv * coloc.row(i)));
    Eigen::MatrixXd Mi2(2, coloc_du.cols());
    Mi2 << Mi;
    Mdense.row(i) = Mi.row(0);
    Mdense.row(i + npts) = Mi.row(1);
  }
  M = Mdense;
}
}
