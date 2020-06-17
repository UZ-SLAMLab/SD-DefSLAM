#include <list>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "ORBVocabulary.h"

#include "DeformationKeyFrame.h"
#include "DeformationORBmatcher.h"

#include "Schwarp.h"
#include "SchwarpDatabase.h"

namespace defSLAM
{
  void SchwarpDatabase::add(KeyFrame *mpCurrentKeyFrame)
  {
    // Check a keyframe is not inserted twice
    CHECK(mpkeyframes.count(mpCurrentKeyFrame) == 0);
    Timer timer;
    timer.start();
    // If it's the first keyframe don't do anything
    if (mpkeyframes.size() == 0)
    {
      mpkeyframes.insert(mpCurrentKeyFrame);
      return;
    }

    // Get matched points
    const vector<MapPoint *> vpMapPointMatches =
        mpCurrentKeyFrame->GetMapPointMatches();

    // Get keyframes of reference for the map points matched
    std::unordered_map<KeyFrame *, int> countKFMatches;

    for (size_t i = 0; i < vpMapPointMatches.size(); i++)
    {
      MapPoint *mapPoint = vpMapPointMatches[i];
      if (!mapPoint)
        continue;
      if (mapPoint->isBad())
        continue;
      // Keyframe in which the point was created.
      KeyFrame *refkf = mapPoint->GetReferenceKeyFrame();
      if (countKFMatches.count(refkf) == 0)
        countKFMatches[refkf] = 0;
      countKFMatches[refkf]++;
    }

    // copy key-value pairs from the map to the vector
    // std::pair<KeyFrame *, int>
    std::vector<std::pair<KeyFrame *, int>> vec;
    std::copy(countKFMatches.begin(),
              countKFMatches.end(),
              std::back_inserter<std::vector<std::pair<KeyFrame *, int>>>(vec));

    uint numberOfKeyframes(7);
    if (vec.size() >= numberOfKeyframes)
    {
      std::nth_element(vec.begin(), vec.begin(), vec.begin() + numberOfKeyframes,
                       [](const std::pair<KeyFrame *, int> &l, const std::pair<KeyFrame *, int> &r) {
                         if (l.second != r.second)
                           return l.second > r.second;

                         return l.first > r.first;
                       });
    }

    std::cout << "list of covisibles :" << std::endl;
    for (uint i(0); i < vec.size() && i < numberOfKeyframes; i++)
    {
      const std::pair<KeyFrame *, int> kf = vec[i];
      std::cout << kf.first << " " << kf.second << std::endl;
    }
    /**/
    /// We search all the possible matches with the reference keyframes.
    /// there will be more points
    for (uint i(0); i < vec.size() && i < numberOfKeyframes; i++)
    {
      const std::pair<KeyFrame *, int> kv = vec[i];
      // Only take into account those with more than 30 matches
      KeyFrame *refkf = kv.first;
      // Get matched points between keyframes
      vector<pair<size_t, size_t>> vMatchedIndices;
      for (size_t i = 0; i < vpMapPointMatches.size(); i++)
      {
        MapPoint *mapPoint = vpMapPointMatches[i];
        if (!mapPoint)
          continue;
        if (mapPoint->isBad())
          continue;
        /// Check that the point is in both keyframes
        if (mapPoint->IsInKeyFrame(mpCurrentKeyFrame) &&
            (mapPoint->IsInKeyFrame(refkf)))
        {
          const int idx1 = mapPoint->GetIndexInKeyFrame(refkf);
          const int idx2 = mapPoint->GetIndexInKeyFrame(mpCurrentKeyFrame);
          vMatchedIndices.push_back({idx1, idx2});
        }
      }
      if (vMatchedIndices.size() < 5)
        continue;
      // Create Schwarp
      double x[_NumberOfControlPointsU * _NumberOfControlPointsV * 2];
      //  DeformationORBmatcher Matcher;
      // Matcher.FindbySchwarp(refkf, mpCurrentKeyFrame, vMatchedIndices, x, reg_);
      this->CalculateSchwarps(refkf, mpCurrentKeyFrame, vMatchedIndices, x, reg_,
                              true);
      static_cast<DeformationKeyFrame *>(mpCurrentKeyFrame)->KeyframesRelated++;
    }
    int count(0);
    for (auto mp : newInformation_)
    {
      if (mp.second)
      {
        count++;
      }
    }
    std::cout << "TOTAL POINTS TO REESTIMATE: " << count << std::endl;
    timer.stop();
    std::cout << timer.getElapsedTimeInMilliSec() << std::endl;
    ofstream myfile;
    myfile.open("final" + std::to_string(uint(mpkeyframes.size())) + ".txt");
    for (auto &point : this->mapPointsDB_)
    {
      myfile << point.second.size() << std::endl;
    }

    myfile.close();
    mpkeyframes.insert(mpCurrentKeyFrame);
  }

  void SchwarpDatabase::clear() { mpkeyframes.clear(); }

  void SchwarpDatabase::erase(KeyFrame *mpCurrentKeyFrame)
  {
    mpkeyframes.erase(mpCurrentKeyFrame);
  }

  void SchwarpDatabase::CalculateSchwarps(
      KeyFrame *KFi, KeyFrame *KF2i,
      vector<pair<size_t, size_t>> &vMatchedIndices,
      double (&x)[_NumberOfControlPointsU * _NumberOfControlPointsV * 2],
      double reg, bool SaveWarp)
  {
    // Calculate From One To Two
    DeformationKeyFrame *KF = static_cast<DeformationKeyFrame *>(KFi);
    DeformationKeyFrame *KF2 = static_cast<DeformationKeyFrame *>(KF2i);
    std::cout << KF2 << std::endl;
    std::vector<cv::KeyPoint> KP1;
    std::vector<cv::KeyPoint> KP2, kp2un;
    std::vector<float> invSigmas;

    KP1.reserve(vMatchedIndices.size());
    KP2.reserve(vMatchedIndices.size());
    kp2un.reserve(vMatchedIndices.size());
    double umin = KF->umin;
    double umax = KF->umax;
    double vmin = KF->vmin;
    double vmax = KF->vmax;

    for (uint ikp = 0; ikp < vMatchedIndices.size(); ikp++)
    {
      const auto &idx1 = vMatchedIndices[ikp].first;
      const auto &idx2 = vMatchedIndices[ikp].second;

      cv::KeyPoint kp1 = KF->mpKeypointNorm[idx1];
      cv::KeyPoint kp2 = KF2->mpKeypointNorm[idx2];
      cv::KeyPoint kp2un_i = KF2->mvKeysUn[idx2];

      KP1.push_back(std::move(kp1));
      KP2.push_back(std::move(kp2));
      kp2un.push_back(std::move(kp2un_i));
      const cv::KeyPoint &kpUn = KF->mvKeysUn[idx1];

      const float &invSigma2 = KF->mvInvLevelSigma2[kpUn.octave];
      invSigmas.push_back(sqrt(invSigma2));
    }

    Eigen::MatrixXd ControlPointsInitialKF(
        _NumberOfControlPointsU * _NumberOfControlPointsV, 2);
    uint us(0);
    for (int i(0); i < KF->NCu; i++)
    {
      for (int j(0); j < KF->NCv; j++)
      {
        ControlPointsInitialKF(us, 0) =
            double((KF->umax - KF->umin) * i) / (KF->NCu - 1) + KF->umin;
        ControlPointsInitialKF(us, 1) =
            double((KF->vmax - KF->vmin) * j) / (KF->NCv - 1) + KF->vmin;
        us++;
      }
    }
    Warps::Warp::initialize(KP1, KP2, reg, KF->umin, KF->umax, KF->vmin, KF->vmax,
                            KF->NCu, KF->NCv, KF->valdim, x);
    ceres::CostFunction *Rep =
        new Warps::Warp(KP1, KP2, invSigmas, umin, umax, vmin, vmax, KF->NCu,
                        KF->NCv, KF->valdim, double(KF->fy), double(KF->fx));
    /// Analitical implementation
    ceres::CostFunction *Schwarzian =
        new Warps::Schwarzian(reg, umin, umax, vmin, vmax, KF->valdim);

    ceres::Problem problem;
    problem.AddParameterBlock(x, _NumberOfControlPointsU *
                                     _NumberOfControlPointsV * 2);
    problem.AddResidualBlock(Rep, new ceres::HuberLoss(5.77), x);
    problem.AddResidualBlock(Schwarzian, nullptr, x);

    // Run the solver!
    ceres::Solver::Options options;
    options.dynamic_sparsity = true;
    options.num_threads = 1;
    options.max_num_iterations = 3;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;

    Solve(options, &problem, &summary);

    Eigen::Matrix<double, _NumberOfControlPointsU * _NumberOfControlPointsV, 2>
        ControlPointsinitial;

    us = 0;
    for (int i(0); i < KF->NCu; i++)
    {
      for (int j(0); j < KF->NCv; j++)
      {
        ControlPointsinitial(us, 0) =
            double((umax - umin) * i) / (KF->NCu - 1) + umin;
        ControlPointsinitial(us, 1) =
            double((vmax - vmin) * j) / (KF->NCv - 1) + vmin;
        us++;
      }
    }

    Eigen::MatrixXd ControlPoints(
        _NumberOfControlPointsU * _NumberOfControlPointsV, 2);

    if (SaveWarp)
    {
      uint us(0);
      for (int i(0); i < KF->NCu; i++)
      {
        for (int j(0); j < KF->NCv; j++)
        {
          ControlPoints(us, 0) = double((umax - umin) * i) / (KF->NCu - 1) + umin;
          ControlPoints(us, 1) = double((vmax - vmin) * j) / (KF->NCv - 1) + vmin;
          us++;
        }
      }
      std::vector<cv::KeyPoint> KP2_e =
          Warps::Warp::getEstimates(KP1, umin, umax, vmin, vmax, KF->NCu, KF->NCv,
                                    KF->valdim, x, ControlPoints, 0, 0);

      Eigen::MatrixXd A;
      A.resize(3, ControlPointsinitial.rows());
      A << ControlPointsinitial.transpose(),
          Eigen::MatrixXd::Ones(1, ControlPointsinitial.rows());

      Eigen::MatrixXd Aend;
      Aend.resize(3, ControlPointsinitial.rows());

      Aend << ControlPoints.transpose(),
          Eigen::MatrixXd::Ones(1, ControlPoints.rows());

      Eigen::Matrix3d K;
      K << KF->fx, 0, KF->cx, 0, KF->fy, KF->cy, 0, 0, 1;
      Eigen::MatrixXd U, Uinitial, Uest;
      Uinitial.resize(3, ControlPointsinitial.rows());
      U.resize(3, ControlPointsinitial.rows());
      Uinitial = K * A;
      U = K * Aend;

      Eigen::MatrixXd Keypoints1, KeypointsEst;
      Keypoints1.resize(3, KP1.size());
      KeypointsEst.resize(3, KP1.size());

      for (uint i(0); i < KP1.size(); i++)
      {
        Keypoints1(0, i) = KP1[i].pt.x;
        KeypointsEst(0, i) = KP2_e[i].pt.x;
        Keypoints1(1, i) = KP1[i].pt.y;
        KeypointsEst(1, i) = KP2_e[i].pt.y;
        Keypoints1(2, i) = 1;
        KeypointsEst(2, i) = 1;
      }

      Keypoints1 = K * Keypoints1;
      KeypointsEst = K * KeypointsEst;

      cv::Mat Init = KF->KFimage.clone();
      cv::Mat end = KF2->KFimage.clone();

      for (uint i(0); i < KP1.size(); i++)
      {
        const MapPoint *pMP1 = KF->GetMapPoint(vMatchedIndices[i].first);
        // If there is already a MapPoint skip
        if (pMP1)
        {
          cv::Scalar yellow(135, 206, 235);
          cv::Scalar red(41, 37, 204);

          // Do not draw outliers.(> px)
          cv::circle(Init, cv::Point(Keypoints1(0, i), Keypoints1(1, i)), 3,
                     yellow, 2);
          cv::circle(end, cv::Point(KeypointsEst(0, i), KeypointsEst(1, i)), 3,
                     yellow, 2);
          cv::circle(end, cv::Point(kp2un[i].pt.x, kp2un[i].pt.y), 2, yellow, 2);
          cv::line(end, cv::Point(KeypointsEst(0, i), KeypointsEst(1, i)),
                   cv::Point(kp2un[i].pt.x, kp2un[i].pt.y), yellow);

          if (cv::norm(cv::Point(KeypointsEst(0, i), KeypointsEst(1, i)) -
                       cv::Point(kp2un[i].pt.x, kp2un[i].pt.y)) > 10)
          {
            cv::circle(Init, cv::Point(Keypoints1(0, i), Keypoints1(1, i)), 3,
                       red, 2);
            cv::circle(end, cv::Point(kp2un[i].pt.x, kp2un[i].pt.y), 2, red, 2);
          }
        }
        else
        {
          cv::Scalar green(81, 150, 62);
          cv::circle(Init, cv::Point(Keypoints1(0, i), Keypoints1(1, i)), 3,
                     green, 2);
          cv::circle(end, cv::Point(KeypointsEst(0, i), KeypointsEst(1, i)), 1,
                     green, 1);
          cv::circle(end, cv::Point(KeypointsEst(0, i), KeypointsEst(1, i)), 5,
                     green, 2);
        }
      }

      int lateral(100);
      int downup(100);

      cv::Mat final(
          cv::Size(Init.cols + end.cols + lateral * 3, Init.rows + 2 * downup),
          CV_8UC3, cv::Scalar(255, 255, 255));
      cv::Mat roi1(final, cv::Rect(lateral, downup, Init.cols, Init.rows));
      cv::Mat roi2(final,
                   cv::Rect(Init.cols + lateral * 2, downup, end.cols, end.rows));

      Init.copyTo(roi1);
      end.copyTo(roi2);
      cv::RNG rng(12345);

      for (uint i(0); i < ControlPointsinitial.rows(); i++)
      {
        cv::Scalar blue(112, 25, 25);  // midnightblue
        cv::Scalar blue2(255, 191, 0); // deepskyblue
        cv::circle(final,
                   cv::Point(Uinitial(0, i) + lateral, Uinitial(1, i) + downup),
                   3, blue, 3);
        cv::circle(final,
                   cv::Point(U(0, i) + end.cols + lateral * 2, U(1, i) + downup),
                   3, blue2, 3);
      }
      cv::imwrite("final" + std::to_string(uint(KF->mnFrameId)) + "-" +
                      std::to_string(uint(KF2->mnFrameId)) + ".png",
                  final);
    }

    std::vector<cv::KeyPoint> qe =
        Warps::Warp::getEstimates(KP1, umin, umax, vmin, vmax, KF->NCu, KF->NCv,
                                  KF->valdim, x, ControlPoints, 0, 0);
    // first order derivatives
    std::vector<cv::KeyPoint> dqu =
        Warps::Warp::getEstimates(KP1, umin, umax, vmin, vmax, KF->NCu, KF->NCv,
                                  KF->valdim, x, ControlPoints, 1, 0);
    std::vector<cv::KeyPoint> dqv =
        Warps::Warp::getEstimates(KP1, umin, umax, vmin, vmax, KF->NCu, KF->NCv,
                                  KF->valdim, x, ControlPoints, 0, 1);
    // second order derivatives
    std::vector<cv::KeyPoint> dquv =
        Warps::Warp::getEstimates(KP1, umin, umax, vmin, vmax, KF->NCu, KF->NCv,
                                  KF->valdim, x, ControlPoints, 1, 1);
    std::vector<cv::KeyPoint> dquu =
        Warps::Warp::getEstimates(KP1, umin, umax, vmin, vmax, KF->NCu, KF->NCv,
                                  KF->valdim, x, ControlPoints, 2, 0);
    std::vector<cv::KeyPoint> dqvv =
        Warps::Warp::getEstimates(KP1, umin, umax, vmin, vmax, KF->NCu, KF->NCv,
                                  KF->valdim, x, ControlPoints, 0, 2);
    uint counter(0);
    uint counter2(0);

    for (size_t ikp = 0; ikp < vMatchedIndices.size(); ikp++)
    {
      const size_t &idx1 = vMatchedIndices[ikp].first;
      const size_t &idx2 = vMatchedIndices[ikp].second;
      MapPoint *mapPoint = KF->GetMapPoint(idx1);
      MapPoint *mapPoint2 = KF2->GetMapPoint(idx2);
      if (!mapPoint)
      {
        continue;
      }
      if (!mapPoint2)
      {
        continue;
      }
      if (mapPoint->isBad())
      {
        continue;
      }
      if (mapPoint2->isBad())
      {
        continue;
      }
      auto error_i = qe[ikp].pt - KP2[ikp].pt;
      error_i.x *= KF->fx;
      error_i.y *= KF->fy;
      counter2++;

      if (cv::norm(error_i) > 20)
      {
        mapPoint2->EraseObservation(KF2);
        KF2->EraseMapPointMatch(idx2);
        continue;
      }
      /// All map points are used for the warp estimation, but only those
      /// whose reference keyframe is the estimated one are saved.
      /*auto pKfref = mapPoint->GetReferenceKeyFrame();
    if (pKfref != KFi)
      continue;*/
      mapPointsDB_[mapPoint].push_back(std::shared_ptr<DiffProp>(new DiffProp()));
      std::shared_ptr<DiffProp> diffVect = mapPointsDB_[mapPoint].back();
      diffVect->KFToKF = std::pair<KeyFrame *, KeyFrame *>(KFi, KF2i);
      diffVect->idx1 = size_t(idx1);
      diffVect->idx2 = size_t(idx2);
      diffVect->I1u = KP1[ikp].pt.x;
      diffVect->I1v = KP1[ikp].pt.y;
      diffVect->I2u = KP2[ikp].pt.x;
      diffVect->I2v = KP2[ikp].pt.y;
      diffVect->J21a = dqv[ikp].pt.y / (dqu[ikp].pt.x * dqv[ikp].pt.y -
                                        dqv[ikp].pt.x * dqu[ikp].pt.y);
      diffVect->J21b = -dqu[ikp].pt.y / (dqu[ikp].pt.x * dqv[ikp].pt.y -
                                         dqv[ikp].pt.x * dqu[ikp].pt.y);
      diffVect->J21c = -dqv[ikp].pt.x / (dqu[ikp].pt.x * dqv[ikp].pt.y -
                                         dqv[ikp].pt.x * dqu[ikp].pt.y);
      diffVect->J21d = dqu[ikp].pt.x / (dqu[ikp].pt.x * dqv[ikp].pt.y -
                                        dqv[ikp].pt.x * dqu[ikp].pt.y);
      diffVect->J12a = dqu[ikp].pt.x;
      diffVect->J12b = dqu[ikp].pt.y;
      diffVect->J12c = dqv[ikp].pt.x;
      diffVect->J12d = dqv[ikp].pt.y;
      diffVect->H12uux = dquu[ikp].pt.x;
      diffVect->H12uuy = dquu[ikp].pt.y;
      diffVect->H12uvx = dquv[ikp].pt.x;
      diffVect->H12uvy = dquv[ikp].pt.y;
      diffVect->H12vvx = dqvv[ikp].pt.x;
      diffVect->H12vvy = dqvv[ikp].pt.y;
      newInformation_[mapPoint] = true;
      counter++;
    }
    std::cout << "Points to reestimate : " << counter << " " << counter2
              << std::endl;
  }

  bool SchwarpDatabase::getDiffPropVector(KeyFrame *KFi, KeyFrame *KFi2, uint k,
                                          DiffProp &DiffReq)
  {
    if (DiffDatabase_.count(std::pair<KeyFrame *, KeyFrame *>(KFi, KFi2)) > 0)
    {
      if (!(DiffDatabase_[std::pair<KeyFrame *, KeyFrame *>(KFi, KFi2)].size() >
            0))
      {
        return false;
      }
      if (DiffDatabase_[std::pair<KeyFrame *, KeyFrame *>(KFi, KFi2)][k])
      {
        DiffReq = *DiffDatabase_[std::pair<KeyFrame *, KeyFrame *>(KFi, KFi2)][k];
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }

} // namespace defSLAM
