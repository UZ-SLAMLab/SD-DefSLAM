#include <PolySolver.h>
#include <iostream>
int main (){
  double a= 1.35;
  double b= 2.45;
  double c= 3.15;
  double d= -2.45;
  double t1= 1.85;
  double t2= -4.85;
  double e1= 1.25;
  double e2= 2.25;
  double x1 = 1.25;
  double y1 = 3.15;
  double x2 = -4.225;
  double y2 = -5.55;

  double eq[10];
  defSLAM::PolySolver::IntroduceParameter2(a,b,c,d,t1,
                                           t2,e1,e2,x1,
                                           y1,x2,y2,0,eq);
  Eigen::Map<Eigen::Matrix<double,10,1>> equ(eq);

  double eq2[10];
  defSLAM::PolySolver::IntroduceParameter2(a,b,c,d,t1,
                                           t2,e1,e2,x1,
                                           y1,x2,y2,1,eq2);
  Eigen::Map<Eigen::Matrix<double,10,1>> eqv(eq2);


  std::cout << equ.transpose() << std::endl;
  std::cout << eqv.transpose() << std::endl;

  return 0;
}
