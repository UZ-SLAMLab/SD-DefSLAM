#include <PolySolver.h>
#include <math.h>

namespace defSLAM {

void PolySolver::introduceParameter(double a, double b, double c, double d,
                                    double t1, double t2, double e1, double e2,
                                    double x1, double y1, double x2, double y2,
                                    int i, double *eq) {
  double detJ12 = a * d - c * b;
  // Eq1
  if (i == 0) {
    /// J12 = [[a,c],[b,d]]
    double eq1_3_0 =
        detJ12 * (t1 * e1 * e2 - detJ12 * (e1 * (c * x2 + d * y2) - y1 * e2));
    double eq1_2_1 =
        -detJ12 * (t2 * e1 * e2 - detJ12 * (e1 * (a * x2 + b * y2) - x1 * e2));
    double eq1_2_0 =
        t2 * (e1 * e2 * t1 -
              detJ12 * (x2 * e1 * c + y2 * e1 * d - 2 * e2 * y1)) -
        t1 * detJ12 * (x2 * e1 * a + e1 * b * y2 + 2 * e2 * x1) +
        detJ12 * detJ12 *
            (e1 * (a * c + b * d) -
             2 * (a * x2 * y1 - c * x1 * x2 + b * y1 * y2 - d * x1 * y2));
    double eq1_1_2 = 0;
    double eq1_0_2 = 0;
    double eq1_0_3 = 0;
    double eq1_1_1 = (e1 * (-e2 * pow(t2, 2) + 2 * x2 * t2 * a * detJ12 +
                            2 * y2 * t2 * b * detJ12 -
                            (pow(a, 2) + pow(b, 2)) * detJ12 * detJ12) +
                      e2 * detJ12 * detJ12);
    double eq1_1_0 =
        t1 * (e2 * detJ12 + 2 * a * x1 * x2 * (detJ12) +
              2 * x1 * y2 * b * detJ12) -
        t2 * 2 *
            (e2 * x1 * t1 +
             detJ12 * (x2 * y1 * a - c * x1 * x2 + y1 * y2 * b - x1 * y2 * d)) +
        e2 * y1 * t2 * t2 +
        detJ12 * detJ12 * (-2 * x1 * (a * c + b * d) + y1 * (a * a + b * b) -
                           c * x2 - d * y2);
    double eq1_0_1 =
        t2 * (detJ12 * (e2 - 2 * a * x1 * x2 - 2 * b * x1 * y2)) +
        x1 * e2 * t2 * t2 +
        (detJ12 * detJ12) * (-y2 * b - x2 * a + x1 * (a * a + b * b));
    double eq1_0_0 = t2 * (e2 * t1 - detJ12 * (c * x2 + d * y2)) -
                     t1 * (detJ12 * (a * x2 + b * y2)) +
                     (a * c + b * d) * detJ12 * detJ12;
    eq[0] = eq1_3_0;
    eq[1] = eq1_2_1;
    eq[2] = eq1_1_2;
    eq[3] = eq1_0_3;
    eq[4] = eq1_2_0;
    eq[5] = eq1_1_1;
    eq[6] = eq1_0_2;
    eq[7] = eq1_1_0;
    eq[8] = eq1_0_1;
    eq[9] = eq1_0_0;
  } else {
    // Eq2
    double eq2_0_3 =
        detJ12 * (e1 * e2 * t2 - (detJ12 * (e1 * (a * x2 + b * y2) - e2 * x1)));
    double eq2_1_2 =
        -detJ12 * (e1 * e2 * t1 - detJ12 * (e1 * (c * x2 + d * y2) - e2 * y1));
    double eq2_2_0 = 0;
    double eq2_3_0 = 0;
    double eq2_2_1 = 0;
    double eq2_0_2 =
        t2 * (e1 * e2 * t1 -
              detJ12 * (e1 * c * x2 + e1 * d * y2 + 2 * e2 * y1)) -
        t1 * detJ12 * (e1 * (a * x2 + b * y2) - 2 * e2 * x1) +
        detJ12 * detJ12 *
            ((e1 * (a * c + b * d) +
              2 * (a * x2 * y1 - c * x1 * x2 + b * y1 * y2 - d * x1 * y2)));
    double eq2_1_1 =
        e1 * (-e2 * t1 * t1 + (detJ12 * (-(c * c + d * d) * detJ12 +
                                         2 * t1 * c * x2 + 2 * d * y2 * t1))) +
        e2 * detJ12 * detJ12;
    double eq2_1_0 = t1 * detJ12 * (e2 - 2 * c * x2 * y1 - 2 * d * y1 * y2) +
                     y1 * (e2 * t1 * t1 + detJ12 * detJ12 * (c * c + d * d)) -
                     detJ12 * detJ12 * (c * x2 + d * y2);
    double eq2_0_1 =
        t2 * (e2 * detJ12 + 2 * y1 * detJ12 * (c * x2 + d * y2)) +
        t1 * (-2 * e2 * y1 * t2 +
              2 * detJ12 *
                  (a * x2 * y1 - c * x1 * x2 + b * y1 * y2 - d * x1 * y2)) +
        e2 * x1 * t1 * t1 -
        2 * detJ12 * detJ12 * (a * c * y1 + 0.5 * a * x2 - 0.5 * c * c * x1 +
                               b * d * y1 + 0.5 * b * y2 - 0.5 * d * d * x1);
    double eq2_0_0 = t2 * (e2 * t1 - detJ12 * (c * x2 + d * y2)) -
                     t1 * (detJ12 * (a * x2 + b * y2)) +
                     detJ12 * detJ12 * (a * c + b * d);
    eq[0] = eq2_3_0;
    eq[1] = eq2_2_1;
    eq[2] = eq2_1_2;
    eq[3] = eq2_0_3;
    eq[4] = eq2_2_0;
    eq[5] = eq2_1_1;
    eq[6] = eq2_0_2;
    eq[7] = eq2_1_0;
    eq[8] = eq2_0_1;
    eq[9] = eq2_0_0;
  }
}
}
