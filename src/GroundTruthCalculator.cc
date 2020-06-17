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

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <algorithm>
#include "GroundTruthCalculator.h"
#include "Timer.h"
#include <Eigen/Dense>
#include <fstream>
using namespace std;

namespace defSLAM {
namespace GroundTruthTools {

/**********************
 * The funtion to generate random floats between a and b 
 * *******************/
float RandomFloat(float a, float b);

/**********************
 * The funtion ScaleMinMedian return the scale estimated
 * for two point clouds with a min median filter
 * 
 * Arg:
 *  PosMono std::vector<std::vector<floats>> points to estimated by DefSLAM
 *  PosStereo std::vector<std::vector<floats>> points estimated by stereo
 * 
 * Return:
 *  Best scale: best estimation of the scale
 * 
 * Author: Javier Morlana
 * 
 * *******************/
float scaleMinMedian(
    std::vector<std::vector<float>> &PosMono,
    std::vector<std::vector<float>> &PosStereo) {

  float min_med = 10000.0;
  int n_points = PosMono.size();
  int final_points = 0;
  double best_scale(0.0);
  Timer timer;
  timer.start();
  for (int i = 0; i < n_points; i++) {
    double r_i = ((double)rand() / (RAND_MAX));

    if (r_i > 0.25)
      continue;
    double scale =
        PosStereo[i][2] / PosMono[i][2]; // stereo_z/mono_z

    vector<float> squared_res;
    squared_res.resize(n_points);
    {
      for (int j = 0; j < n_points; j++) {
        squared_res[j] = -1;
        if (i == j)
          continue;
        double r_j = ((double)rand() / (RAND_MAX));

        if (r_j > 0.25)
          continue;
        float r2(0.0);
        for (uint k(0);k<3;k++){
           auto res = (scale * PosMono[j][k] - PosStereo[j][k]);
           r2 = r2+res*res;
        }
        squared_res[j] = std::sqrt(r2);
      }
    }

    // Get the median of the residual
    std::sort(squared_res.begin(), squared_res.end());

    int NumberNonZero(0);

    while (squared_res[NumberNonZero++] < 0)
      continue;

    std::vector<float> NonZeroVec(squared_res.begin() + NumberNonZero,
                                  squared_res.end());

    int median_index = round(NonZeroVec.size() / 2);
    final_points++;

    if (NonZeroVec.size() == 0)
      return 0.0;

    if (NonZeroVec[median_index] < min_med) {
      // Save the minimum median
      min_med = NonZeroVec[median_index];
      best_scale = scale;
    }

    squared_res.clear();
  }

  timer.stop();

  std::cout << " final points and min median " << final_points << " " << min_med
            << " (" << timer.getElapsedTimeInMilliSec() << "ms)" << std::endl;

  // Desviation
  float desv = 1.4826 * (1.0 - (5.0 / (final_points - 1.0))) * sqrt(min_med);
  std::cout << " stan dev " << desv << std::endl;

  // Comparison and rejecting outliers
  std::vector<int> NonOutlier;
  for (int i = 0; i < n_points; i++) {
    float residual(0.0);
    for (uint j(0);j<3;j++){
           auto rest = (best_scale * PosMono[i][j] - PosStereo[i][j]);
           residual = residual+rest*rest;
    }
    residual = std::sqrt(residual);
     
    if ((residual / desv) < 50) {
      NonOutlier.push_back(i);
    }
  }

  n_points = NonOutlier.size();

  // Final scale with inliers
  float Sum_num = 0.0;
  float Sum_den = 0.0;

  for (int i = 0; i < n_points; i++) {
    int indx = NonOutlier[i];
    Sum_num += (PosStereo[indx][2] * PosMono[indx][2]);
    Sum_den += (PosMono[indx][2] * PosMono[indx][2]);
  }

  best_scale = Sum_num / Sum_den;
  return best_scale;
}

 /**********************
 * The funtion SaveResults saves a vector in a text file
 * 
 * Author: Javier Morlana
 * 
 * Arg:
 *  vectorError std::vector<floats> save a vector 
 *  name std::string name for the file
 * 
 * Return:
 *  Best scale: best estimation of the scale
 * 
 * *******************/ 
void saveResults(std::vector<float> &vectorError,
                                        std::string &name) {
  Eigen::MatrixXd asd(vectorError.size(), 1);
  for (uint i(0); i < vectorError.size(); i++) {
    asd(i, 0) = vectorError[i];
  }
  std::ofstream myfile;
  myfile.open(name);
  myfile << asd;
  myfile.close();
}
} // namespace GroundTruthTools
} // namespace defSLAM
