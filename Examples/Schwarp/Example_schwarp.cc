#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <Thirdparty/Schwarps/Schwarp.h>
#include <Timer.h>
#include <iostream>
#include <fstream>
#include <ceres/ceres.h>
using namespace std;

int main() {
    //Warps::Schwarp Scharp;
    ifstream myReadFile;
    myReadFile.open("/home/jose/ORB-SDTAM/Examples/FeaturesMatched1017.txt");
    char output[100];
    std::vector<cv::KeyPoint> KP1,KP2;

    if (myReadFile.is_open()) {
        uint i(0);
        double aux[3];
        while (!myReadFile.eof()) {
            myReadFile >> output;
       //     std::cout << output << std::endl;

            if (i == 3){
                cv::KeyPoint keypoint1,keypoint2;
                keypoint1.pt.x = aux[0];
                keypoint1.pt.y = aux[1];
                keypoint2.pt.x = aux[2];


                keypoint2.pt.y = atof(output);
                KP1.push_back(keypoint1);
                KP2.push_back(keypoint2);
                i=0;
            }else {
               // std::cout << output << std::endl;

                aux[i] = atof(output);
                i++;
            }
        }
   }
   myReadFile.close();

   std::cout << "Program STARTED" << std::endl;
   double umin=-1; double umax=1; double vmin=-1; double vmax=1; int NCu=_NumberOfControlPointsU; int NCv=_NumberOfControlPointsV; int valdim=2;
   Warps::Schwarp(KP1,KP2,1e-1,umin,umax,vmin,vmax,NCu,NCv,valdim);
   return 0;
}
