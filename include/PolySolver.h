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
#ifndef POLYSOLVER_H
#define POLYSOLVER_H
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <iostream>

namespace defSLAM {

int gelimd2(double **a, double *b, double *x, int n);
void fww(double *x, double *fv, Eigen::MatrixXd &eqs1, Eigen::MatrixXd &eqs2,
         int n);
struct Eqs {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 1, 10> eqs1;
  Eigen::Matrix<double, 1, 10> eqs2;
};

class PolySolver : public ceres::SizedCostFunction<2, 2> {
public:
  PolySolver(const Eigen::Matrix<double, 1, 10> &eq1,
             const Eigen::Matrix<double, 1, 10> &eq2) {
    init_d(eq1, eq2);
  }
  ~PolySolver() { delete eqs; }
  bool Evaluate(double const *const *parameters, double *e,
                double **Jac) const {
    Eigen::Matrix<double, 1, 10> eqs1 = eqs->eqs1;
    Eigen::Matrix<double, 1, 10> eqs2 = eqs->eqs2;
    uint i(0);
    const double *x = parameters[0];
    e[0] = eqs1(i, 0) * pow(x[0], 3) +
           eqs1(i, 1) * pow(x[0], 2) * pow(x[1], 1) +
           eqs1(i, 2) * pow(x[0], 1) * pow(x[1], 2) +
           eqs1(i, 3) * pow(x[1], 3) + eqs1(i, 4) * pow(x[0], 2) +
           eqs1(i, 5) * x[0] * x[1] + eqs1(i, 6) * pow(x[1], 2) +
           eqs1(i, 7) * x[0] + eqs1(i, 8) * x[1] + eqs1(i, 9);
    e[1] = eqs2(i, 0) * pow(x[0], 3) +
           eqs2(i, 1) * pow(x[0], 2) * pow(x[1], 1) +
           eqs2(i, 2) * pow(x[0], 1) * pow(x[1], 2) +
           eqs2(i, 3) * pow(x[1], 3) + eqs2(i, 4) * pow(x[0], 2) +
           eqs2(i, 5) * x[0] * x[1] + eqs2(i, 6) * pow(x[1], 2) +
           eqs2(i, 7) * x[0] + eqs2(i, 8) * x[1] + eqs2(i, 9);
    if (Jac) {
      const double *x = parameters[0];
      Jac[0][0] = 3 * eqs1(i, 0) * pow(x[0], 2) +
                  2 * eqs1(i, 1) * pow(x[0], 1) * pow(x[1], 1) +
                  eqs1(i, 2) * pow(x[1], 2) + 2 * eqs1(i, 4) * pow(x[0], 1) +
                  eqs1(i, 5) * x[1] + eqs1(i, 7);
      Jac[0][1] = eqs1(i, 1) * pow(x[0], 2) +
                  2 * eqs1(i, 2) * pow(x[0], 1) * pow(x[1], 1) +
                  3 * eqs1(i, 3) * pow(x[1], 2) + eqs1(i, 5) * x[0] +
                  2 * eqs1(i, 6) * pow(x[1], 1) + eqs1(i, 8);
      Jac[0][2] = 3 * eqs2(i, 0) * pow(x[0], 2) +
                  2 * eqs2(i, 1) * pow(x[0], 1) * pow(x[1], 1) +
                  eqs2(i, 2) * pow(x[1], 2) + 2 * eqs2(i, 4) * pow(x[0], 1) +
                  eqs2(i, 5) * x[1] + eqs2(i, 7);
      Jac[0][3] = eqs2(i, 1) * pow(x[0], 2) +
                  2 * eqs2(i, 2) * pow(x[0], 1) * pow(x[1], 1) +
                  3 * eqs2(i, 3) * pow(x[1], 2) + eqs2(i, 5) * x[0] +
                  2 * eqs2(i, 6) * pow(x[1], 1) + eqs2(i, 8);
    }
    return true;
  }

  static void introduceParameter(double a, double b, double c, double d,
                                 double t1, double t2, double e, double e1,
                                 double x1, double y1, double x2, double y2,
                                 int i, double *eq);

private:
  void init_d(const Eigen::Matrix<double, 1, 10> &eq1,
              const Eigen::Matrix<double, 1, 10> &eq2) {
    eqs = new Eqs;
    eqs->eqs1 = eq1;
    eqs->eqs2 = eq2;
  }
  Eqs *eqs;
};
}
#endif
