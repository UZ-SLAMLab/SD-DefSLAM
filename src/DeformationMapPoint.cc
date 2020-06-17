/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
* of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "DeformationMapPoint.h"
#include "ORBmatcher.h"

#include <mutex>

namespace defSLAM
{
  class DiffProp;

  DeformationMapPoint::DeformationMapPoint(const cv::Mat &Pos, KeyFrame *pRefKF,
                                           Map *pMap)
      : MapPoint(Pos, pRefKF, pMap), mpTemplate(static_cast<Template *>(nullptr)),
        facet(static_cast<Facet *>(nullptr)), Deformable(false),
        InsertInTheTemplate(false), nofacet(false) {}

  DeformationMapPoint::DeformationMapPoint(const cv::Mat &Pos, KeyFrame *pRefKF,
                                           Map *pMap, cv::Point2f obs, cv::Vec3f normal)
      : MapPoint(Pos, pRefKF, pMap, obs), mpTemplate(static_cast<Template *>(nullptr)),
        facet(static_cast<Facet *>(nullptr)), Deformable(false),
        InsertInTheTemplate(false), nofacet(false) {}

  DeformationMapPoint::DeformationMapPoint(const cv::Mat &Pos, Map *pMap,
                                           Frame *pFrame, const int &idxF)
      : MapPoint(Pos, pMap, pFrame, idxF),
        mpTemplate(static_cast<Template *>(nullptr)),
        facet(static_cast<Facet *>(nullptr)), Deformable(false), nofacet(false) {}

  void DeformationMapPoint::AssignedTemplate(const Template *temp)
  {
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    mpTemplate = temp;
  }

  void DeformationMapPoint::RemoveTemplate()
  {
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    if (mpTemplate)
    {
      mpTemplate = static_cast<Template *>(nullptr);
      facet = static_cast<Facet *>(nullptr);
      for (uint i(0); i < Nodes.size(); i++)
        Nodes[i] = static_cast<Node *>(nullptr);
    }
  }

  cv::Mat DeformationMapPoint::GetWorldPosAtRest()
  {
    std::set<Node *> Nodes = facet->getNodes();
    std::vector<Node *> Nodesvector;
    for (std::set<Node *>::iterator itn = Nodes.begin(); itn != Nodes.end();
         itn++)
    {
      Nodesvector.push_back(*itn);
    }
    cv::Mat mWorldPosAtRest = mWorldPos.clone();
    mWorldPosAtRest.at<float>(0) = b1 * Nodesvector[0]->xO +
                                   b2 * Nodesvector[1]->xO +
                                   b3 * Nodesvector[2]->xO;
    mWorldPosAtRest.at<float>(1) = b1 * Nodesvector[0]->yO +
                                   b2 * Nodesvector[1]->yO +
                                   b3 * Nodesvector[2]->yO;
    mWorldPosAtRest.at<float>(2) = b1 * Nodesvector[0]->zO +
                                   b2 * Nodesvector[1]->zO +
                                   b3 * Nodesvector[2]->zO;
    return mWorldPosAtRest.clone();
  }

  void DeformationMapPoint::SetBadFlag()
  {
    map<KeyFrame *, size_t> obs;
    {
      unique_lock<mutex> lock1(mMutexFeatures);
      unique_lock<mutex> lock2(mMutexPos);
      mbBad = true;
      obs = mObservations;
      mObservations.clear();
    }
    for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end();
         mit != mend; mit++)
    {
      KeyFrame *pKF = mit->first;
      pKF->EraseMapPointMatch(mit->second);
    }

    mpMap->EraseMapPoint(this);

    this->SetNoFacet();
  }

  void DeformationMapPoint::SetFacet(Facet *face)
  {
    if (face)
      face->addMapPoint(this);
    facet = face;
    std::unique_lock<std::mutex> MutexNce(MutexNFace);
    nofacet = false;
  }

  Facet *DeformationMapPoint::getFacet()
  {
    std::unique_lock<std::mutex> MutexNce(MutexFacet);
    return facet;
  }
  void DeformationMapPoint::SetCoordinates(double B1, double B2, double B3)
  {
    b1 = B1;
    b2 = B2;
    b3 = B3;
  }

  void DeformationMapPoint::SetRGB(double r, double g, double b)
  {
    this->RGB[0] = r;
    this->RGB[1] = g;
    this->RGB[2] = b;
  }

  void DeformationMapPoint::getRGB(double &r, double &g, double &b)
  {
    r = this->RGB[0];
    g = this->RGB[1];
    b = this->RGB[2];
  }

  void DeformationMapPoint::Repose()
  {
    this->RecalculatePosition();
    this->UpdateNormalAndDepth();
  }

  void DeformationMapPoint::RecalculatePosition()
  {
    std::set<Node *> Nodes = facet->getNodes();
    std::vector<Node *> Nodesvector;
    for (std::set<Node *>::iterator itn = Nodes.begin(); itn != Nodes.end();
         itn++)
    {
      Nodesvector.push_back(*itn);
    }

    // unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);

    mWorldPos.at<float>(0) =
        b1 * Nodesvector[0]->x + b2 * Nodesvector[1]->x + b3 * Nodesvector[2]->x;
    mWorldPos.at<float>(1) =
        b1 * Nodesvector[0]->y + b2 * Nodesvector[1]->y + b3 * Nodesvector[2]->y;
    mWorldPos.at<float>(2) =
        b1 * Nodesvector[0]->z + b2 * Nodesvector[1]->z + b3 * Nodesvector[2]->z;
  }

  bool DeformationMapPoint::CheckReliability(Facet *Candidatefacet, double B1,
                                             double B2, double B3)
  {
    this->SetCoordinates(B1, B2, B3);
    std::set<Node *> Nodes = Candidatefacet->getNodes();
    std::vector<Node *> Nodesvector;
    for (std::set<Node *>::iterator itn = Nodes.begin(); itn != Nodes.end();
         itn++)
    {
      Nodesvector.push_back(*itn);
    }

    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mWorldPos.at<float>(0) =
        b1 * Nodesvector[0]->x + b2 * Nodesvector[1]->x + b3 * Nodesvector[2]->x;
    mWorldPos.at<float>(1) =
        b1 * Nodesvector[0]->y + b2 * Nodesvector[1]->y + b3 * Nodesvector[2]->y;
    mWorldPos.at<float>(2) =
        b1 * Nodesvector[0]->z + b2 * Nodesvector[1]->z + b3 * Nodesvector[2]->z;

    vector<double> distmean;
    for (std::map<KeyFrame *, size_t>::iterator itkF = mObservations.begin();
         itkF != mObservations.end(); itkF++)
    {
      cv::KeyPoint kP = (itkF->first)->mvKeys[itkF->second];

      cv::Mat Tcw = (itkF->first)->GetPose();
      cv::Mat Temp = mWorldPos.clone();
      cv::Mat row = cv::Mat::ones(1, 1, CV_32F);
      Temp.push_back(row);
      cv::Mat mCamPos = Tcw * Temp;
      double x = (itkF->first)->fx * mCamPos.at<float>(0) +
                 (itkF->first)->cx * mCamPos.at<float>(2);
      double y = (itkF->first)->fy * mCamPos.at<float>(1) +
                 (itkF->first)->cy * mCamPos.at<float>(2);
      double v = mCamPos.at<float>(2);
      x = x / v;
      y = y / v;
      double x2 = (x - kP.pt.x) * (x - kP.pt.x);
      double y2 = (y - kP.pt.y) * (y - kP.pt.y);
      distmean.push_back(sqrt(x2 + y2) / mObservations.size());
    }

    std::sort(distmean.begin(), distmean.end());
    if ((distmean[uint(distmean.size() / 2)] < 5))
    {
      this->SetFacet(Candidatefacet);
      return true;
    }
    else
    {
      return false;
    }
  }

  bool DeformationMapPoint::CheckFacet(Facet *face)
  {
    std::unique_lock<std::mutex> MutexNce(MutexFacet);

    if (!(facet))
    {
      this->SetFacet(face);
      return true;
    }
    if (face == facet)
      return true;
    else
      return false;
  }
  void DeformationMapPoint::UpdateNormalAndDepth()
  {
    if (!facet)
    {
      MapPoint::UpdateNormalAndDepth();
      return;
    }
    map<KeyFrame *, size_t> observations;
    KeyFrame *pRefKF;
    cv::Mat Pos;
    {
      unique_lock<mutex> lock1(mMutexFeatures);
      unique_lock<mutex> lock2(mMutexPos);
      if (mbBad)
        return;
      observations = mObservations;
      pRefKF = mpRefKF;
      Pos = mWorldPos.clone();
    }

    if (observations.empty())
      return;

    int n = 0;
    std::vector<Eigen::Vector3f> posesNode;
    for (auto &node : facet->getNodes())
    {
      double x, y, z;
      node->getXYZ(x, y, z);
      Eigen::Vector3f pose;
      pose << x, y, z;
      posesNode.push_back(std::move(pose));
    }

    Eigen::Vector3f vector1 = posesNode[1] - posesNode[2];
    Eigen::Vector3f vector2 = posesNode[3] - posesNode[2];
    Eigen::Vector3f normal = vector1.cross(vector2);
    auto lastNormal = mNormalVector.clone();
    normal.normalize();
    cv::eigen2cv(normal, mNormalVector);

    const float cosParallaxRays =
        mNormalVector.dot(lastNormal) / (cv::norm(mNormalVector) * cv::norm(lastNormal));
    if (cosParallaxRays < 0)
    {
      mNormalVector = -mNormalVector;
    }
    cv::Mat PC = Pos.clone() - pRefKF->GetCameraCenter().clone();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor = pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
      unique_lock<mutex> lock3(mMutexPos);
      mfMaxDistance = dist * levelScaleFactor;
      mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
    }
  }
  bool DeformationMapPoint::thereisface()
  {
    std::unique_lock<std::mutex> MutexNce(MutexNFace);
    return !nofacet;
  }

  void DeformationMapPoint::SetNoFacet()
  {
    std::unique_lock<std::mutex> MutexNce(MutexNFace);
    nofacet = true;
    mpTemplate = nullptr;
  }

  void DeformationMapPoint::AddPositioninK(KeyFrame *kf, cv::Mat &X)
  {
    PosesKeyframes[kf] = X.clone();
  }
  void DeformationMapPoint::SetWorldPos(cv::Mat x3d, KeyFrame *kf)
  {
    PosesKeyframes[kf] = x3d;
  }

} // namespace defSLAM
