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

#include "GroundTruthKeyFrame.h"
#include "CC_MAC.h"
#include "DeformationMap.h"
#include "DeformationMapPoint.h"
#include "DeformationMapPoint.h"
#include "DeformationORBmatcher.h"
#include "Facet.h"
#include "Facet.h"
#include "GroundTruthCalculator.h"
#include "MapPoint.h"
#include "ORBmatcher.h"
#include "PolySolver.h"
#include "diffProp.h"
#include <PCLNormalEstimator.h>
#include <math.h>
#include <mutex>
#include <opencv2/core/core.hpp>

namespace defSLAM {

GroundTruthKeyFrame::GroundTruthKeyFrame(Frame &F, Map *pMap,
                                         KeyFrameDatabase *pKFDB)
    : DeformationKeyFrame(F, pMap, pKFDB), imRight(F.imRight.clone()),
      StereoAvaliable(false), tempx_(TEMPX), tempy_(TEMPY), margin_(MARGIN),
      searchx_(SEARCHX), searchy_(tempy_ + margin_), threshold_(THRESHOLD),
      radious_(RADIOUS) {
  if (F.StereoAvailable) {
    this->StereoAvaliable = true;
  }
}

float GroundTruthKeyFrame::Estimate3DScale() {
  size_t nval = this->mvKeysUn.size();
  cv::Mat iGry(this->imGray.clone());
  std::vector<int> ptreal;
  std::vector<float> points_est;
  std::vector<float> points_gt;
  std::vector<float> normals_est;
  posMono_.reserve(nval);
  posStereo_.reserve(nval);
  posMono_.clear();
  posStereo_.clear();
  std::vector<double *> covariances;
  for (uint i = 0; i < nval; i++) {
    MapPoint *pMP = this->GetMapPoint(i);
    if (!pMP)
      continue;
    if (pMP->isBad())
      continue;
    cv::Vec3f pos;

    this->surface->Get3DSurfacePoint(i, pos);
    cv::Mat PosMat(4, 1, CV_32F);
    for (uint i(0); i < 3; i++)
      PosMat.at<float>(i) = pos(i);

    pos(3) = 1;

    cv::KeyPoint kp = this->mvKeysUn[i];

    int tempx(tempx_), tempy(tempy_);
    int searchx(searchx_), searchy(searchy_);

    if (((kp.pt.x - tempx / 2) < 0) or ((kp.pt.y - tempy / 2) < 0) or
        ((kp.pt.x + tempx / 2) > this->imRight.cols) or
        ((kp.pt.y + tempy / 2) > this->imRight.rows))
      continue;

    cv::Rect cropRect(kp.pt.x - tempx / 2, kp.pt.y - tempy / 2, tempx, tempy);
    int finx = (kp.pt.x - searchx / 2);
    int finy = (kp.pt.y - searchy);
    int finxright = finx + searchx;
    int finyright = finy + searchy * 2;
    if (finx < 0)
      finx = 0;
    if (finy < 0)
      finy = 0;
    if (finxright > this->imRight.cols)
      searchx = float(this->imRight.cols - 1 - finx);
    if (finyright > this->imRight.rows)
      searchy = float(this->imRight.rows - 1 - finy) / 2;

    cv::Rect SearchRegion(finx, finy, searchx, searchy * 2);

    cv::Mat tmp = iGry(cropRect);
    cv::Mat SearchImage = this->imRight(SearchRegion);

    cv::Mat result;
    cv::matchTemplate(SearchImage, tmp, result, cv::TM_CCORR_NORMED);

    cv::Point matchLoc;

    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

    double CorrelationThreshold(threshold_);

    if (maxVal < CorrelationThreshold)
      continue;

    matchLoc = maxLoc;

    matchLoc.x = matchLoc.x + finx + tempx / 2;
    matchLoc.y = matchLoc.y + finy + tempy / 2;
    
    float disp(std::abs(matchLoc.x - kp.pt.x));
    float z = mbf / disp / 2;
    float mindisp(mbf);
    if ((z > mbf / 2) or (z < (mindisp / fx))) {
      continue;
    }

    {
      cv::Vec3f normal;
      // For normal estimation
      if (this->surface->GetNormalSurfacePoint(i, normal)) {
        for (uint i(0); i < 3; i++)
          normals_est.push_back(normal(i));
        points_gt.push_back(mbf / disp * (((float)kp.pt.x - cx) / fx));
        points_gt.push_back(mbf / disp * (((float)kp.pt.y - cy) / fy));
        points_gt.push_back(mbf / disp);
        points_est.push_back(PosMat.at<float>(0));
        points_est.push_back(PosMat.at<float>(1));
        points_est.push_back(PosMat.at<float>(2));

        std::vector<float> pm;
        pm.push_back(PosMat.at<float>(0));
        pm.push_back(PosMat.at<float>(1));
        pm.push_back(PosMat.at<float>(2));
        posMono_.push_back(pm);
        std::vector<float> ps;
        ps.push_back(mbf / disp * (((float)kp.pt.x - cx) / fx));
        ps.push_back(mbf / disp * (((float)kp.pt.y - cy) / fy));
        ps.push_back(mbf / disp);
        posStereo_.push_back(ps);
        covariances.push_back(pMP->covNorm);
        ptreal.push_back(i);
      }
    }
  }
  if (points_gt.size() == 0) {
    std::cout << "Points" << std::endl;
    return 1;
  }
  PCLNormalEstimator pne(points_gt, radious_);
  std::vector<float> normals_gt = pne.getNormals();
  PCLNormalEstimator pne2(points_est, radious_);
  std::vector<float> normals_sfn = pne2.getNormals();

  std::vector<float> ErrorAngleIso;
  std::vector<float> ErrorAngleSfN;

  double sumIso(0.0);
  double sumSfN(0.0);

  for (uint i(0); i < normals_gt.size() / 3; i++) {
    // std::cout << "Normal " << i << std::endl;
    Eigen::Vector3f nest;
    nest << normals_est[3 * i + 0], normals_est[3 * i + 1],
        normals_est[3 * i + 2];
    Eigen::Vector3f nreal;
    nreal << normals_gt[3 * i + 0], normals_gt[3 * i + 1],
        normals_gt[3 * i + 2];
    Eigen::Vector3f nsft;
    nsft << normals_sfn[3 * i + 0], normals_sfn[3 * i + 1],
        normals_sfn[3 * i + 2];
    nest.normalize();
    nreal.normalize();
    nsft.normalize();
    double aIso = std::acos((nest.transpose() * nreal)) * 180 / M_PI;
    double aSfN = std::acos(nreal.transpose() * nsft) * 180 / M_PI;

    if (aIso > 90)
      aIso = 180 - aIso;

    Eigen::Map<Eigen::Matrix<double, 2, 2>> cov(covariances[i]);
    // std::cout << cov << "  " << aIso << " " << nest.transpose() << "-" <<
    // nreal.transpose()<< std::endl;
    if (aSfN > 90)
      aSfN = 180 - aSfN;
    //  std::cout << "Estimated by SfN " << aSfN << std::endl;
    //   std::cout << "Cov : " << cov.eigenvalues() << std::endl;

    if (aIso == aIso) {
      ErrorAngleIso.push_back(aIso);
      sumIso += aIso;
    }
    if (aSfN == aSfN) {
      ErrorAngleSfN.push_back(aSfN);
      sumSfN += aSfN;
    }
  }
  if (ErrorAngleIso.size() < 1)
    return 1;
  std::accumulate(ErrorAngleIso.begin(), ErrorAngleIso.end(), sumIso);
  std::sort(ErrorAngleIso.begin(), ErrorAngleIso.end());

  std::cout << "Mean angle Iso Error Keyframe : " << std::endl
            << "min : "
            << *std::min_element(ErrorAngleIso.begin(), ErrorAngleIso.end())
            << std::endl
            << "max : "
            << *std::max_element(ErrorAngleIso.begin(), ErrorAngleIso.end())
            << std::endl
            << "median : " << ErrorAngleIso[ErrorAngleIso.size() / 2]
            << std::endl
            << sumIso / ((double)ErrorAngleIso.size()) << " " << std::endl;

  // std::cout << "Mean angle SfN Error Keyframe : " <<
  // sumSfN/((double)ErrorAngleSfN.size()) << " " << std::endl;
  static int is(0);
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(5)
      << uint(this->mTimeStamp);
  std::string name("ErrorAngIso" + out.str() + "-" + std::to_string(is) +
                   ".txt");
  std::string name2("ErrorAngSfN" + out.str() + ".txt");
  is++;
  GroundTruthTools::saveResults(ErrorAngleIso, name);
  GroundTruthTools::saveResults(ErrorAngleSfN, name2);

  float Scale = GroundTruthTools::scaleMinMedian(posMono_, posStereo_);
  std::cout << "Scale Corr stereo Keyframe: " << Scale << std::endl;
  return Scale;
}

void GroundTruthKeyFrame::Estimate3DLocalMap(float s) {
  cv::Mat iGry(this->imGray.clone());
  cv::Mat iRight(this->imRight.clone());

  std::vector<float> Error;
  int count(0);
  double acc(0.0);

  this->mvLocalMapPoints.clear();
  this->mvStereoMapPoints.clear();

  for (size_t i = 0; i < posMono_.size(); i++) {
    cv::Mat Pc(4, 1, CV_32F);
    //  Pc = this->Tcw*PosMat;
    const float &PcX = posMono_[i][0];
    const float &PcY = posMono_[i][1];
    const float &PcZ = posMono_[i][2];
    cv::Mat Pest(Pc.clone());
    Pest.at<float>(0) = posStereo_[i][0];
    Pest.at<float>(1) = posStereo_[i][1];
    Pest.at<float>(2) = posStereo_[i][2];
    float er = float(sqrt(pow(Pest.at<float>(0) - s * PcX, 2) +
                          pow(Pest.at<float>(1) - s * PcY, 2) +
                          pow(Pest.at<float>(2) - s * PcZ, 2)));
    if (er == er) {
      Error.push_back(er);
      acc += er;
    }
    cv::Point3f local(PcX, PcY, PcZ);
    cv::Point3f stereo(Pest.at<float>(0), Pest.at<float>(1), Pest.at<float>(2));

    this->mvLocalMapPoints.push_back(local);
    this->mvStereoMapPoints.push_back(stereo);
  }
  if (Error.size() == 0)
    return;
  double sum(0.0);
  std::sort(Error.begin(), Error.end());

  std::accumulate(Error.begin(), Error.end(), sum);
  double invc = 1 / ((double)count);
  std::cout << "Mean Error Keyframe : " << acc * invc << " " << Error.size()
            << " " << std::endl;
  std::cout << "Median Error Keyframe : " << Error[Error.size() / 2] << " "
            << std::endl;

  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(5)
      << uint(this->mTimeStamp);
  std::string name("ErrorKFs" + out.str() + ".txt");
  GroundTruthTools::saveResults(Error, name);
}

void GroundTruthKeyFrame::Estimate3DLocalMapIso(Map *map, double s) {
  // const vector<MapPoint*> &vpRefMPs = map->GetReferenceMapPoints();
  const vector<MapPoint *> &vpMPs = map->GetAllMapPoints();

  // set<MapPoint*> spRefMPs(vpMPs.begin(), vpMPs.end());

  cv::Mat iGry(this->imGray.clone());
  cv::Mat iRight(this->imRight.clone());

  std::vector<cv::KeyPoint> kp1, kp2;
  std::vector<cv::DMatch> matches;
  std::vector<std::pair<int, int>> aMatches;
  std::vector<float> Error;
  int count(0);
  double acc(0.0);
  int nval = this->mvKeysUn.size();

  this->mvLocalMapPoints.clear();
  this->mvStereoMapPoints.clear();

  // 3D estimation
  for (int i = 0; i < nval; i++) {
    // OpenCV_Template Matching tutorial :
    // https://docs.opencv.org/master/de/da9/tutorial_template_matching.html
    cv::Vec3f pos;

    this->surface->Get3DSurfacePoint(i, pos);
    cv::Mat PosMat(4, 1, CV_32F);
    for (uint i(0); i < 3; i++)
      PosMat.at<float>(i) = pos(i);

    pos(3) = 1;
    cv::KeyPoint kp = this->mvKeysUn[i];

    if (kp.pt.x < 0)
      continue;

    // cv::circle(iGry,kp.pt,3,cv::Scalar(254,0,0));

    int tempx(11), tempy(11), margin(2);
    int searchx(80), searchy(tempy + margin);

    if (((kp.pt.x - tempx) < 0) or ((kp.pt.y - tempy) < 0) or
        ((kp.pt.x + tempx) > iRight.cols) or ((kp.pt.y + tempy) > iRight.rows))
      continue;

    if (((kp.pt.x - searchx * 2) < 0) or ((kp.pt.y - searchy) < 0) or
        ((kp.pt.x) > iRight.cols) or ((kp.pt.y + searchy) > iRight.rows))
      continue;

    cv::Rect cropRect(kp.pt.x - tempx, kp.pt.y - tempy, tempx * 2, tempy * 2);
    int finx = (kp.pt.x - searchx * 2);

    if (finx < 0)
      finx = 0;

    cv::Rect SearchRegion(finx, kp.pt.y - searchy, searchx * 2, searchy * 2);

    cv::Mat tmp = iGry(cropRect);
    cv::Mat SearchImage = iRight(SearchRegion);

    cv::Mat result;
    cv::matchTemplate(SearchImage, tmp, result, cv::TM_CCORR_NORMED);
    // PointsSeen(pos)

    // cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
    cv::Point matchLoc;

    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

    double CorrelationThreshold(0.95);

    if (maxVal < CorrelationThreshold)
      continue;

    matchLoc = maxLoc;

    /*  cv::imshow("SearchImage",SearchImage);
      cv::imshow("tmp",tmp);*/

    matchLoc.x = matchLoc.x + kp.pt.x - searchx * 2 + tempx;
    matchLoc.y = matchLoc.y + kp.pt.y - searchy + tempy;

    kp1.push_back(kp);
    kp2.push_back(cv::KeyPoint(matchLoc.x, matchLoc.y, 1));

    aMatches.push_back(std::make_pair(count, count));

    double disp(std::abs(matchLoc.x - kp.pt.x));
    double z = mbf / disp;

    if ((z > 1) or (z < (mbf / fx))) {
      continue;
    }
    cv::Mat Pc(4, 1, CV_32F);
    //  Pc = this->Tcw*PosMat;
    const float &PcX = PosMat.at<float>(0);
    const float &PcY = PosMat.at<float>(1);
    const float &PcZ = PosMat.at<float>(2);
    cv::Mat Pest(Pc.clone());
    Pest.at<float>(0) = mbf / disp * (((double)kp.pt.x - cx) / fx);
    Pest.at<float>(1) = mbf / disp * (((double)kp.pt.y - cy) / fy);
    Pest.at<float>(2) = mbf / disp;
    double er = sqrt(pow(Pest.at<float>(0) - s * PcX, 2) +
                     pow(Pest.at<float>(1) - s * PcY, 2) +
                     pow(Pest.at<float>(2) - s * PcZ, 2));
    if (er == er) {
      Error.push_back(er);
      acc += er;
    }
    cv::Point3f local(PcX, PcY, PcZ);
    cv::Point3f stereo(Pest.at<float>(0), Pest.at<float>(1), Pest.at<float>(2));

    this->mvLocalMapPoints.push_back(local);
    this->mvStereoMapPoints.push_back(stereo);

    count++;
  }
  if (Error.size() == 0)
    return;
  double sum(0.0);
  std::sort(Error.begin(), Error.end());

  std::accumulate(Error.begin(), Error.end(), sum);
  double invc = 1 / ((double)count);
  std::cout << "Mean Error Keyframe Iso: " << acc * invc << " " << Error.size()
            << " " << std::endl;
  std::cout << "Median Error Keyframe Iso: " << Error[Error.size() / 2] << " "
            << std::endl;

  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(5)
      << uint(this->mTimeStamp);
  std::string name("ErrorKFIsos" + out.str() + ".txt");
  GroundTruthTools::saveResults(Error, name);
}

} // namespace DefSLAM
