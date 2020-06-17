#include "SurfaceRegistration.h"
#include "Converter.h"
#include "DeformationKeyFrame.h"
#include "DeformationMapPoint.h"
#include "DeformationOptimizer.h"
#include "Surface.h"
#include "SurfacePoint.h"
namespace defSLAM {
using ORB_SLAM2::MapPoint;
using ORB_SLAM2::Converter;

SurfaceRegistration::SurfaceRegistration(KeyFrame *Keyframe, double chiLimit_,
                                         bool check_chi)
    : refKF(Keyframe), chiLimit_(chiLimit_), check_chi(check_chi) {}

SurfaceRegistration::~SurfaceRegistration() { refKF = nullptr; }

bool SurfaceRegistration::Register() {
  std::vector<std::vector<float>> cloud1pc;
  std::vector<std::vector<float>> cloud2pc;
  cv::Mat Twc = refKF->GetPoseInverse();
  cv::Mat TcwKf = refKF->GetPose();

  cloud1pc.reserve(refKF->mvKeys.size());
  cloud2pc.reserve(refKF->mvKeys.size());

  uint CounterMP(0);
  for (uint i(0); i < refKF->mvKeys.size(); i++) {
    MapPoint *pMP = refKF->GetMapPoint(i);
    if (pMP) {
      if (pMP->isBad())
        continue;

      if (static_cast<DeformationMapPoint *>(pMP)->getFacet()) {
        if (!pMP->covNorm)
          continue;
        cv::Mat x3DMap(4, 1, CV_32F);
        cv::Mat x3Dy = static_cast<DeformationMapPoint *>(pMP)
                           ->PosesKeyframes[refKF]
                           .clone();
        if (x3Dy.empty())
          continue;
        std::vector<float> pt1;
        pt1.push_back(x3Dy.at<float>(0, 0));
        pt1.push_back(x3Dy.at<float>(1, 0));
        pt1.push_back(x3Dy.at<float>(2, 0));
        x3DMap.at<float>(0, 0) = x3Dy.at<float>(0, 0);
        x3DMap.at<float>(1, 0) = x3Dy.at<float>(1, 0);
        x3DMap.at<float>(2, 0) = x3Dy.at<float>(2, 0);
        x3DMap.at<float>(3, 0) = 1;
        cloud1pc.push_back(std::move(pt1));
        cv::Vec3f x3DsurfKf;
        static_cast<DeformationKeyFrame *>(refKF)->surface->Get3DSurfacePoint(
            i, x3DsurfKf);
        cv::Mat x3DSurW(4, 1, CV_32F);
        x3DSurW.at<float>(0, 0) = x3DsurfKf(0);
        x3DSurW.at<float>(1, 0) = x3DsurfKf(1);
        x3DSurW.at<float>(2, 0) = x3DsurfKf(2);
        x3DSurW.at<float>(3, 0) = 1;
        cv::Mat x3DKf = (Twc)*x3DSurW;
        std::vector<float> pt2;
        pt2.push_back(x3DKf.at<float>(0, 0));
        pt2.push_back(x3DKf.at<float>(1, 0));
        pt2.push_back(x3DKf.at<float>(2, 0));
        cloud2pc.push_back(std::move(pt2));
        CounterMP++;
      }
    }
  }

  if (CounterMP < 15)
    return false;
  float scale = ScaleMinMedian(cloud2pc, cloud1pc);
  Eigen::Matrix4f TwcEigen;
  cv::cv2eigen(Twc, TwcEigen);

  Eigen::Matrix4f transform_;
  transform_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  Eigen::Matrix4d transform_d;
  for (uint i(0); i < 4; i++) {
    for (uint j(0); j < 4; j++) {
      transform_d(i, j) = transform_(i, j);
    }
  }

  g2o::Sim3 transf(transform_d.block(0, 0, 3, 3), transform_d.block(0, 3, 3, 1),
                   scale);

  double Huber(0.01);
  double chi(pow(chiLimit_, 2));
  bool aceptable =
      Optimizer::OptimizeHorn(cloud2pc, cloud1pc, transf, chi, Huber);
  std::cout << "acceptable " << chi << " " << aceptable << " " << check_chi
            << std::endl;
  if ((!aceptable) && (check_chi))
    return false;

  cv::Mat mScw = Converter::toCvMat(transf);
  Eigen::MatrixXf aasdasd;
  cv::cv2eigen(mScw, aasdasd);
  TwcEigen = aasdasd * TwcEigen;

  Eigen::MatrixXf TT2 =
      TwcEigen.block(0, 0, 3, 3) * (TwcEigen.block(0, 0, 3, 3)).transpose();
  double s22 = std::sqrt(TT2(0, 0));
  static_cast<DeformationKeyFrame *>(refKF)->surface->applyScale(s22);

  TwcEigen.block(0, 0, 3, 3) = TwcEigen.block(0, 0, 3, 3) / (s22);
  Eigen::MatrixXf Tcw = TwcEigen.inverse();
  cv::eigen2cv(Tcw, mScw);
  refKF->SetPose(mScw);

  return true;
}

float SurfaceRegistration::ScaleMinMedian(
    std::vector<std::vector<float>> &PosMono,
    std::vector<std::vector<float>> &PosStereo) {
  using cv::Mat;
  using cv::Mat_;

  float min_med = 10000.0;
  int n_points = PosMono.size();
  int final_points = 0;
  double best_scale(0.0);
  Timer timer;
  timer.start();
  for (int i = 0; i < n_points; i++) {
    Mat XYZ_mono = (Mat_<float>(3, 1) << PosMono[i][0], PosMono[i][1],
                    PosMono[i][2]); // Mono
    Mat XYZ_stereo = (Mat_<float>(3, 1) << PosStereo[i][0], PosStereo[i][1],
                      PosStereo[i][2]); // Stereo

    double scale =
        XYZ_stereo.at<float>(2) / XYZ_mono.at<float>(2); // stereo z/mono z

    vector<float> squared_res;
    squared_res.resize(n_points);
    {
      for (int j = 0; j < n_points; j++) {
        squared_res[j] = -1;
        if (i == j)
          continue;
        /*  if (r_j>0.3)
               continue;*/
        Mat XYZ_mono_ij =
            (Mat_<float>(3, 1) << PosMono[j][0], PosMono[j][1], PosMono[j][2]);
        Mat XYZ_stereo_ij = (Mat_<float>(3, 1) << PosStereo[j][0],
                             PosStereo[j][1], PosStereo[j][2]);
        float r2 = cv::norm(scale * XYZ_mono_ij - XYZ_stereo_ij);
        squared_res[j] = r2;
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
    Mat XYZ_mono = (Mat_<float>(3, 1) << PosMono[i][0], PosMono[i][1],
                    PosMono[i][2]); // Mono
    Mat XYZ_stereo = (Mat_<float>(3, 1) << PosStereo[i][0], PosStereo[i][1],
                      PosStereo[i][2]); // Stereo

    float residual = cv::norm(best_scale * XYZ_mono - XYZ_stereo);
    // std::cout << residual/desv << std::endl;

    if ((residual / desv) < 2.0) {
      NonOutlier.push_back(i);
    }
  }
  // cout<<"Points_erased_in_scale: "<<Points_erased_in_scale<<endl;
  n_points = NonOutlier.size();

  // Final scale with inliers
  float Sum_num = 0.0;
  float Sum_den = 0.0;
  float mean_residual = 0.0;

  for (int i = 0; i < n_points; i++) {
    Mat XYZ_mono =
        (Mat_<float>(3, 1) << PosMono[NonOutlier[i]][0],
         PosMono[NonOutlier[i]][1], PosMono[NonOutlier[i]][2]); // Mono
    Mat XYZ_stereo =
        (Mat_<float>(3, 1) << PosStereo[NonOutlier[i]][0],
         PosStereo[NonOutlier[i]][1], PosStereo[NonOutlier[i]][2]); // Stereo

    mean_residual += sqrt(
        (XYZ_stereo.at<float>(2) - best_scale * XYZ_mono.at<float>(2)) *
            (XYZ_stereo.at<float>(2) - best_scale * XYZ_mono.at<float>(2)) +
        (XYZ_stereo.at<float>(1) - best_scale * XYZ_mono.at<float>(1)) *
            (XYZ_stereo.at<float>(1) - best_scale * XYZ_mono.at<float>(1)) +
        (XYZ_stereo.at<float>(0) - best_scale * XYZ_mono.at<float>(0)) *
            (XYZ_stereo.at<float>(0) - best_scale * XYZ_mono.at<float>(0)));

    Sum_num += (XYZ_stereo.at<float>(2) * XYZ_mono.at<float>(2));
    Sum_den += (XYZ_mono.at<float>(2) * XYZ_mono.at<float>(2));
  }

  best_scale = Sum_num / Sum_den;
  // cout<<"definitive scale: "<<best_scale<<endl;
  return best_scale;
}
} // namespace ORB_SLAM2
