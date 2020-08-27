#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <string>

#include <System.h>
#include <Hamlyn/dataloader.h>
#include <SettingsLoader.h>

using namespace std;

int main(int argc, char **argv)
{
  /*****
     * Usage:
     * ./elasHamlyn <folder_images_Left> <folder_images_Right> <pattern_images_Left> <pattern_images_Right>
     * *****/

  dataset::Hamlyn::DataLoader *hamlynloader;
  auto drawPattern(false); // Activate to generate results with horizontal lines (epipolar lines)
  if (argc == 9)
  {
    string folderImagesLeft(argv[2]);
    string folderImagesRight(argv[3]);
    string patternImagesLeft(argv[4]);
    string patternImagesRight(argv[5]);
    string leftCalibration(argv[6]);
    string rightCalibration(argv[7]);
    string extrinsicCalibration(argv[8]);
    hamlynloader = new dataset::Hamlyn::DataLoader(folderImagesLeft, folderImagesRight, patternImagesLeft,
                                                   patternImagesRight, leftCalibration, rightCalibration,
                                                   extrinsicCalibration, drawPattern);
  }
  else if (argc == 6)
  {
    string videoPair(argv[2]);
    string leftCalibration(argv[3]);
    string rightCalibration(argv[4]);
    string extrinsicCalibration(argv[5]);
    hamlynloader = new dataset::Hamlyn::DataLoader(videoPair, leftCalibration, rightCalibration,
                                                   extrinsicCalibration, drawPattern);
  }
  else if (argc == 7)
  {
    string videoLeft(argv[2]);
    string videoRight(argv[3]);

    string leftCalibration(argv[4]);
    string rightCalibration(argv[5]);
    string extrinsicCalibration(argv[6]);
    hamlynloader = new dataset::Hamlyn::DataLoader(videoLeft, videoRight, leftCalibration, rightCalibration,
                                                   extrinsicCalibration, drawPattern);
  }
  else
  {
    std::cout << "args passed " << argc << std::endl;
    cout << endl;
    cout << endl;
    return 0;
  }

  std::cout << "Number of images: " << hamlynloader->size() << std::endl;

  defSLAM::SettingsLoader settingsLoader;
  settingsLoader.setSaveResults(false);
  cv::Mat distcoeff = settingsLoader.getdistCoef();
  cv::Mat K = hamlynloader->obtainNewCalibrationValues();
  float bf = hamlynloader->obtainBaselineF();
  auto widthAndHeight = hamlynloader->getWidthAndHeight();
  settingsLoader.setbf(bf);
  settingsLoader.setCameraWidth(widthAndHeight.first);
  settingsLoader.setCameraHeight(widthAndHeight.second);
  std::cout << K << std::endl;

  std::cout << settingsLoader.getK() << std::endl;
  settingsLoader.setK(K);

  std::cout << settingsLoader.getdistCoef() << std::endl;
  std::cout << settingsLoader.getbf() << std::endl;

  defSLAM::System SLAM(argv[1], settingsLoader, true);

  for (size_t i(0); i < hamlynloader->size(); i++)
  {
    auto stereoPair = hamlynloader->operator[](i);

    cv::Mat grayLeft, grayRight;

    grayLeft = stereoPair.first.clone();
    grayRight = stereoPair.second.clone();

    cv::Mat _mask(grayLeft.rows, grayLeft.cols, CV_8UC1, cv::Scalar(255));
    clock_t start = clock();
    SLAM.TrackMonocularGT(grayLeft, grayRight, i, _mask);
    clock_t end = clock();

    double diffticks = end - start;
    double diffms = diffticks / (CLOCKS_PER_SEC / 1000);

    std::cout << "time " << diffms << std::endl;
  }

  delete hamlynloader;
  return 0;
}