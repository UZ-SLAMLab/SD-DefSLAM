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

#ifndef LINEAR_DEF
#define LINEAR_DEF

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/dynamic_autodiff_cost_function.h>
#include <ceres/dynamic_numeric_diff_cost_function.h>
#include <ceres/jet.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/sized_cost_function.h>

namespace defSLAM {

struct LSToSolve {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::SparseMatrix<double> LinearSystemSparse;
  Eigen::MatrixXd b;
};

class LinearSystemForCeres {
public:
  LinearSystemForCeres(Eigen::MatrixXd &LS, Eigen::MatrixXd &b)
      : p(LS.cols()), r(LS.rows()) {
    init_d(LS, b);
  }

  virtual ~LinearSystemForCeres() { delete LinearSystem; }

  bool operator()(const double *const parameters, double *residuals) const {

    for (uint k = 0; k < r; k++) {
      residuals[k] = -LinearSystem->b(k, 0);
    }

    for (int k = 0; k < LinearSystem->LinearSystemSparse.outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(
               LinearSystem->LinearSystemSparse, k);
           it; ++it) {
        residuals[it.row()] += it.value() * parameters[it.col()];
      }
    }

    return true;
  }

private:
  void init_d(Eigen::MatrixXd &LS, Eigen::MatrixXd &b) {
    LinearSystem = new LSToSolve;
    LinearSystem->LinearSystemSparse = LS.sparseView();
    LinearSystem->b = b;
  }

  LSToSolve *LinearSystem;
  uint p, r;
};
}

#endif
