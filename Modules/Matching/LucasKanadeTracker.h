/*
 * This file is part of SD-DefSLAM
 * Copyright (C) 2020 Juan J. Gómez Rodríguez, Jose Lamarca Peiro, J. Morlana,
 *                    Juan D. Tardós and J.M.M. Montiel, University of Zaragoza
 *
 * This software is for internal use in the EndoMapper project.
 * Not to be re-distributed.
 */

#ifndef LUCASKANADETRACKER_LUCASKANADETRACKER_H
#define LUCASKANADETRACKER_LUCASKANADETRACKER_H

#include <vector>
#include <opencv2/core/core.hpp>

#include "KeyFrame.h"

/*
 * This class implements several variants of the Lucas-Kanade optical flow algorithm: illuminance invariance,
 * with homographies and with patterns instead of square patches.
 */
class LucasKanadeTracker
{
public:
    /*
     * Default constructor
     */
    LucasKanadeTracker();

    /*
     * Constructor with parameters
     */
    LucasKanadeTracker(const cv::Size _winSize, const int _maxLevel, const int _maxIters,
                       const float _epsilon, const float _minEigThreshold);

    /*
     * Precomputes data for the reference image used for tracking next images
     */

    //This one does not update the patches, it uses  the one extracted the first time a point was seen
    void SetReferenceImage(cv::Mat &refIm, std::vector<ORB_SLAM2::MapPoint *> &refMps, std::vector<cv::KeyPoint> &refPts);

    //This one updates the patches with the current image location
    void SetReferenceImage(cv::Mat &refIm, std::vector<cv::KeyPoint> &refPts);

    void AddFromKeyFrame(ORB_SLAM2::KeyFrame *pKF, std::vector<cv::KeyPoint> &vKeys, std::vector<ORB_SLAM2::MapPoint *> &vMPs);

    /*
     * Tracks a new image using precomputed data. SetReferenceImage must be called
     * at least once before calling this method
     */
    int PRE_Track(cv::Mat &newIm, std::vector<cv::KeyPoint> &nextPts, std::vector<bool> &status, std::vector<cv::Mat>& vHessian,
                const bool bInitialFlow, const float minSSIM);

    int PRE_Track_VEC(cv::Mat &newIm, std::vector<cv::KeyPoint> &nextPts, std::vector<bool> &status, const bool bInitialFlow,
                      const float minSSIM, std::vector<float> &vSSIM, std::vector<cv::Mat> &vOutWins);

    /*
     * KLT with Homographies
     */
    inline void Hwarp(float *pH, const float x, const float y, float &xR, float &yR);
    inline void SubPixWeights(cv::Point2f p, int &w00, int &w01, int &w10, int &w11, int &xRi, int &yRi);

    void ComputeRelativeCoordinates(const std::vector<cv::KeyPoint> &refPts, std::vector<cv::Mat> &vH,
                                    std::vector<std::vector<float>> &vRelCoorsX, std::vector<std::vector<float>> &vRelCoorsY);

    int TrackWithInfoWithHH(cv::Mat &newIm, std::vector<cv::KeyPoint> &nextPts,
                            const std::vector<cv::KeyPoint> &prevPts,
                            std::vector<bool> &status, const std::vector<std::vector<cv::Mat>> vPatches,
                            const std::vector<std::vector<cv::Mat>> vGrad, const std::vector<std::vector<float>> vMean,
                            const std::vector<std::vector<float>> vMean2, std::vector<cv::Mat> &vH, const float minSSIM,
                            std::vector<cv::Mat>& vHessian);

    void AddPointsFromMapPoints(const vector<ORB_SLAM2::MapPoint *> &vpNewMPs,
                                const std::vector<cv::KeyPoint> &vNewKeys, const cv::Mat &im,
                                vector<ORB_SLAM2::MapPoint *> &vpUpdatedMPs,
                                std::vector<cv::KeyPoint> &vUpdatedKeys);

private:
    int interpolate(int &a, int &b, int &c, int &d, int w, int x, int y, int z, int WBITS);

public:
    //-------------------------------
    //        KLT parameters
    //-------------------------------
    cv::Size winSize;      //size of the integration window of the Lucas-Kanade algorithm
    int maxLevel;          //max level of the image pyramids
    int maxIters;          //max number of iterations of the optical flow algorithm
    float epsilon;         //minimum optical flow imposed displacement. If lower, we stop computing
    float minEigThreshold; //min eigen threshold value for the Spatial Gradient matrix

    //-------------------------------
    //      Pre computed stuff
    //-------------------------------
    std::vector<cv::Mat> refPyr;             //Reference pyramid
    std::vector<cv::KeyPoint> prevPts;       //Original coordinates of the points to track
    std::vector<std::vector<float>> vMeanI;  //Reference window mean intensities
    std::vector<std::vector<float>> vMeanI2; //Reference window mean squared intensities
    std::vector<std::vector<cv::Mat>> Iref;  //Reference windows
    std::vector<std::vector<cv::Mat>> Idref; //Reference derivative windows
};

#endif //LUCASKANADETRACKER_LUCASKANADETRACKER_H
