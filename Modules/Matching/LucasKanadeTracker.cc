#include "LucasKanadeTracker.h"

#include <float.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <x86intrin.h>
#include <smmintrin.h>
#include <iostream>

using namespace std;
using namespace cv;

#define CV_MAKETYPE(depth, cn) (CV_MAT_DEPTH(depth) + (((cn)-1) << CV_CN_SHIFT))
#define CV_DESCALE(x, n) (((x) + (1 << ((n)-1))) >> (n))

LucasKanadeTracker::LucasKanadeTracker() : winSize(Size(21, 21)), maxLevel(3), maxIters(30), epsilon(0.01),
                                           minEigThreshold(1e-4)
{
}

LucasKanadeTracker::LucasKanadeTracker(const cv::Size _winSize, const int _maxLevel, const int _maxIters,
                                       const float _epsilon, const float _minEigThreshold) : winSize(_winSize), maxLevel(_maxLevel), maxIters(_maxIters), epsilon(_epsilon),
                                                                                             minEigThreshold(_minEigThreshold)
{
    vMeanI = vector<vector<float>>(maxLevel + 1);
    vMeanI2 = vector<vector<float>>(maxLevel + 1);
    Iref = vector<vector<Mat>>(maxLevel + 1);
    Idref = vector<vector<Mat>>(maxLevel + 1);
}

void LucasKanadeTracker::SetReferenceImage(cv::Mat &refIm, std::vector<ORB_SLAM2::MapPoint *> &refMps, std::vector<cv::KeyPoint> &refPts) {
    //Compute reference pyramid
    cv::buildOpticalFlowPyramid(refIm, refPyr, winSize, maxLevel);

    //Store points
    prevPts = refPts;

    for(int level = maxLevel; level >= 0; level--){
        vMeanI[level].clear();
        vector<float>().swap(vMeanI[level]);
        vMeanI2[level].clear();
        vector<float>().swap(vMeanI2[level]);
        Iref[level].clear();
        vector<cv::Mat>().swap(Iref[level]);
        Idref[level].clear();
        vector<cv::Mat>().swap(Idref[level]);

        vMeanI[level].resize(refMps.size());
        vMeanI2[level].resize(refMps.size());
        Iref[level].resize(refMps.size());
        Idref[level].resize(refMps.size());

        for(int i = 0; i < refMps.size(); i++){
            if(!refMps[i]) continue;
            vMeanI[level][i] = refMps[i]->mvMean[level];
            vMeanI2[level][i] = refMps[i]->mvMean2[level];
            Iref[level][i] = refMps[i]->mvPatch[level].clone();
            Idref[level][i] = refMps[i]->mvGrad[level].clone();
        }
    }
}

void LucasKanadeTracker::SetReferenceImage(Mat &refIm, vector<KeyPoint> &refPts)
{
    //Compute reference pyramid
    cv::buildOpticalFlowPyramid(refIm, refPyr, winSize, maxLevel);

    //Store points
    prevPts = refPts;

    //Compute reference windows (intensity and derivtives) and means of the windows
    Point2f halfWin((winSize.width - 1) * 0.5f, (winSize.height - 1) * 0.5f);

    for (int level = maxLevel; level >= 0; level--)
    {
        vMeanI[level].clear();
        vector<float>().swap(vMeanI[level]);
        vMeanI2[level].clear();
        vector<float>().swap(vMeanI2[level]);
        Iref[level].clear();
        vector<cv::Mat>().swap(Iref[level]);
        Idref[level].clear();
        vector<cv::Mat>().swap(Idref[level]);

        vMeanI[level].resize(refPts.size());
        vMeanI2[level].resize(refPts.size());
        Iref[level].resize(refPts.size());
        Idref[level].resize(refPts.size());

        //Get images form the pyramid
        const Mat I = refPyr[level * 2];
        const Mat derivI = refPyr[level * 2 + 1];

        //Steps for matrix indexing
        int dstep = (int)(derivI.step / derivI.elemSize1());
        int stepI = (int)(I.step / I.elemSize1());

        //Buffer for fast memory access
        int cn = I.channels(), cn2 = cn * 2; //cn should be 1 therefor cn2 should be 2
        AutoBuffer<short> _buf(winSize.area() * (cn + cn2));
        int derivDepth = DataType<short>::depth;

        //Integration window buffers
        Mat IWinBuf(winSize, CV_MAKETYPE(derivDepth, cn), (*_buf));
        Mat derivIWinBuf(winSize, CV_MAKETYPE(derivDepth, cn2), (*_buf) + winSize.area() * cn);

        for (int i = 0; i < prevPts.size(); i++)
        {
            //Compute image coordinates in the reference image at the current level
            Point2f point = prevPts[i].pt / (float)(1 << level);

            Point2i ipoint;
            point -= halfWin;
            ipoint.x = cvFloor(point.x);
            ipoint.y = cvFloor(point.y);

            if (ipoint.x < (float)(-winSize.width) || ipoint.x >= (float)derivI.cols ||
                ipoint.y < (float)(-winSize.height) || ipoint.y >= (float)derivI.rows)
            {
                continue;
            }

            //Compute weighs for sub pixel computation
            float a = point.x - ipoint.x;
            float b = point.y - ipoint.y;
            const int W_BITS = 14, W_BITS1 = 14;
            const float FLT_SCALE = 1.f / (1 << 20);
            int iw00 = cvRound((1.f - a) * (1.f - b) * (1 << W_BITS));
            int iw01 = cvRound(a * (1.f - b) * (1 << W_BITS));
            int iw10 = cvRound((1.f - a) * b * (1 << W_BITS));
            int iw11 = (1 << W_BITS) - iw00 - iw01 - iw10;

            //Compute sumI, sumI2, meanI, meanI2, Iwin, IdWin
            float meanI = 0.f, meanI2 = 0.f;

            int x, y;
            for (y = 0; y < winSize.height; y++)
            {
                //Get pointers to the images
                const uchar *src = I.ptr() + (y + ipoint.y) * stepI + ipoint.x;
                const short *dsrc = derivI.ptr<short>() + (y + ipoint.y) * dstep + ipoint.x * 2;

                //Get pointers to the window buffers
                short *Iptr = IWinBuf.ptr<short>(y);
                short *dIptr = derivIWinBuf.ptr<short>(y);

                x = 0;
                for (; x < winSize.width * cn; x++, dsrc += 2, dIptr += 2)
                {
                    //Get sub pixel values from images
                    int ival = CV_DESCALE(src[x] * iw00 + src[x + cn] * iw01 +
                                              src[x + stepI] * iw10 + src[x + stepI + cn] * iw11,
                                          W_BITS1 - 5);
                    int ixval = CV_DESCALE(dsrc[0] * iw00 + dsrc[cn2] * iw01 +
                                               dsrc[dstep] * iw10 + dsrc[dstep + cn2] * iw11,
                                           W_BITS1);
                    int iyval = CV_DESCALE(dsrc[1] * iw00 + dsrc[cn2 + 1] * iw01 + dsrc[dstep + 1] * iw10 +
                                               dsrc[dstep + cn2 + 1] * iw11,
                                           W_BITS1);

                    //Store values to the window buffers
                    Iptr[x] = (short)ival;
                    dIptr[0] = (short)ixval;
                    dIptr[1] = (short)iyval;

                    //Compute accum values for later gain and bias computation
                    meanI += (float)ival;
                    meanI2 += (float)(ival * ival);
                }
            }
            //Compute means for later gain and bias computation
            vMeanI[level][i] = (meanI * FLT_SCALE) / winSize.area();
            vMeanI2[level][i] = (meanI2 * FLT_SCALE) / winSize.area();

            Iref[level][i] = IWinBuf.clone();
            Idref[level][i] = derivIWinBuf.clone();
        }
    }
}

int LucasKanadeTracker::PRE_Track(Mat &newIm, std::vector<KeyPoint> &nextPts, vector<bool> &status, std::vector<cv::Mat>& vHessian,
                                const bool bInitialFlow, const float minSSIM)
{
    //Dimensions of half of the window
    Point2f halfWin((winSize.width - 1) * 0.5f, (winSize.height - 1) * 0.5f);

    //covariance
    vector<cv::Mat> jac_(nextPts.size());
    for(size_t i = 0; i < jac_.size(); i++){
        jac_[i] = cv::Mat(winSize.area(),2,CV_32F);
    }

    //Compute pyramid images
    vector<Mat> newPyr;
    cv::buildOpticalFlowPyramid(newIm, newPyr, winSize, maxLevel);

    //Start Lucas-Kanade optical flow algorithm
    //First iterate over pyramid levels
    for (int level = maxLevel; level >= 0; level--)
    {
        //Get images and gradients
        const Mat I = refPyr[level * 2];
        const Mat J = newPyr[level * 2];
        const Mat derivI = refPyr[level * 2 + 1];
        const Mat derivJ = newPyr[level * 2 + 1];

        //Buffer for fast memory access
        int j, cn = I.channels(), cn2 = cn * 2; //cn should be 1 therefor cn2 should be 2
        AutoBuffer<short> _buf(winSize.area() * (cn + cn2) * 2);
        int derivDepth = DataType<short>::depth;

        //Integration window buffers
        Mat IWinBuf(winSize, CV_MAKETYPE(derivDepth, cn), (*_buf));
        Mat JWinBuf(winSize, CV_MAKETYPE(derivDepth, cn), (*_buf) + winSize.area());
        Mat derivIWinBuf(winSize, CV_MAKETYPE(derivDepth, cn2), (*_buf) + 2 * winSize.area());
        Mat derivJWinBuf(winSize, CV_MAKETYPE(derivDepth, cn2), (*_buf) + 4 * winSize.area());

        //Steps for matrix indexing
        int dstep = (int)(derivI.step / derivI.elemSize1());
        int stepJ = (int)(J.step / J.elemSize1());

        //Track each point at the current pyramid level
        for (int i = 0; i < prevPts.size(); i++)
        {
            if (!status[i])
                continue;

            //Compute image coordinates in the reference image at the current level
            Point2f prevPt = prevPts[i].pt * (float)(1. / (1 << level));
            //Compute image coordinates in the current frame at the current level
            Point2f nextPt;
            if (level == maxLevel)
            {
                if (bInitialFlow)
                {
                    nextPt = nextPts[i].pt * (float)(1. / (1 << level));
                }
                else
                {
                    nextPt = prevPt;
                }
            }
            else
            {
                nextPt = nextPts[i].pt * 2.f;
            }

            nextPts[i].pt = nextPt;

            //Check that previous point and next point is inside of the
            //image boundaries
            Point2i iprevPt, inextPt;
            prevPt -= halfWin;
            iprevPt.x = cvFloor(prevPt.x);
            iprevPt.y = cvFloor(prevPt.y);

            if (iprevPt.x < (float)(-winSize.width) || iprevPt.x >= (float)derivI.cols ||
                iprevPt.y < (float)(-winSize.height) || iprevPt.y >= (float)derivI.rows)
            {
                if (level == 0)
                    status[i] = false;

                continue;
            }

            //Compute weighs for sub pixel computation
            const int W_BITS = 14, W_BITS1 = 14;
            const float FLT_SCALE = 1.f / (1 << 20);

            //Compute sumI, sumI2, meanI, meanI2, Iwin, IdWin
            float meanI = 0.f, meanI2 = 0.f;

            int x, y;
            //Compute means for later gain and bias computation
            meanI = vMeanI[level][i];
            meanI2 = vMeanI2[level][i];

            IWinBuf = Iref[level][i].clone();
            derivIWinBuf = Idref[level][i].clone();

            //Optical flow loop
            Point2f prevDelta(0.0f, 0.0f);
            nextPt -= halfWin;
            for (j = 0; j < maxIters; j++)
            {
                //Compute weighs for sub pixel computation
                inextPt.x = cvFloor(nextPt.x);
                inextPt.y = cvFloor(nextPt.y);

                //Check that the point is inside the image
                if (inextPt.x < -(float)winSize.width || inextPt.x >= (float)J.cols ||
                    inextPt.y < -(float)winSize.height || inextPt.y >= (float)J.rows)
                {
                    if (level == 0)
                        status[i] = false;
                    break;
                }

                float aJ = nextPt.x - inextPt.x;
                float bJ = nextPt.y - inextPt.y;
                int jw00 = cvRound((1.f - aJ) * (1.f - bJ) * (1 << W_BITS));
                int jw01 = cvRound(aJ * (1.f - bJ) * (1 << W_BITS));
                int jw10 = cvRound((1.f - aJ) * bJ * (1 << W_BITS));
                int jw11 = (1 << W_BITS) - jw00 - jw01 - jw10;

                //Compute alpha and beta for gain and bias
                //Compute sumI, sumI2, meanI, meanI2, Iwin, IdWin
                float meanJ = 0.f, meanJ2 = 0.f;

                for (y = 0; y < winSize.height; y++)
                {
                    //Get pointers to the images
                    const uchar *src = J.ptr() + (y + inextPt.y) * stepJ + inextPt.x * cn;
                    const short *dsrc = derivJ.ptr<short>() + (y + inextPt.y) * dstep + inextPt.x * cn2;

                    //Get pointers to the window buffers
                    short *Jptr = JWinBuf.ptr<short>(y);
                    short *dJptr = derivJWinBuf.ptr<short>(y);

                    x = 0;

                    for (; x < winSize.width * cn; x++, dsrc += 2, dJptr += 2)
                    {
                        //Get sub pixel values from images
                        int jval = CV_DESCALE(src[x] * jw00 + src[x + cn] * jw01 +
                                                  src[x + stepJ] * jw10 + src[x + stepJ + cn] * jw11,
                                              W_BITS1 - 5);
                        int jxval = CV_DESCALE(dsrc[0] * jw00 + dsrc[cn2] * jw01 +
                                                   dsrc[dstep] * jw10 + dsrc[dstep + cn2] * jw11,
                                               W_BITS1);
                        int jyval = CV_DESCALE(dsrc[1] * jw00 + dsrc[cn2 + 1] * jw01 + dsrc[dstep + 1] * jw10 +
                                                   dsrc[dstep + cn2 + 1] * jw11,
                                               W_BITS1);

                        //Store values to the window buffers
                        Jptr[x] = (short)jval;
                        dJptr[0] = (short)jxval;
                        dJptr[1] = (short)jyval;

                        //Compute accum values for later gain and bias computation
                        meanJ += (float)jval;
                        meanJ2 += (float)(jval * jval);
                    }
                }

                //Compute means for later gain and bias computation
                meanJ = (meanJ * FLT_SCALE) / winSize.area();
                meanJ2 = (meanJ2 * FLT_SCALE) / winSize.area();

                //Compute alpha and beta
                float alpha = sqrt(meanI2 / meanJ2);
                float beta = meanI - alpha * meanJ;

                //Compute image gradient insensitive to ilumination changes
                float ib1 = 0, ib2 = 0;
                float b1, b2;
                float iA11 = 0, iA12 = 0, iA22 = 0;
                float A11, A12, A22;

                int rr = 0;

                for (y = 0; y < winSize.height; y++)
                {
                    //Get pointers to the buffers
                    const short *Iptr = IWinBuf.ptr<short>(y);
                    const short *Jptr = JWinBuf.ptr<short>(y);
                    const short *dIptr = derivIWinBuf.ptr<short>(y);
                    const short *dJptr = derivJWinBuf.ptr<short>(y);

                    x = 0;
                    for (; x < winSize.width * cn; x++, dIptr += 2, dJptr += 2)
                    {
                        int diff = Jptr[x] * alpha - Iptr[x] - beta;
                        float dx = (float)(dIptr[0] + dJptr[0] * alpha);
                        float dy = (float)(dIptr[1] + dJptr[1] * alpha);

                        ib1 += (float)(diff * dx);
                        ib2 += (float)(diff * dy);

                        iA11 += (float)(dx * dx);
                        iA22 += (float)(dy * dy);
                        iA12 += (float)(dx * dy);

                        jac_[i].at<float>(rr,0) = (float) (diff * dx);
                        jac_[i].at<float>(rr,1) = (float) (diff * dy);
                        rr++;
                    }
                }
                b1 = ib1 * FLT_SCALE;
                b2 = ib2 * FLT_SCALE;

                jac_[i] *= FLT_SCALE;

                //Compute spatial gradient matrix
                A11 = iA11 * FLT_SCALE;
                A12 = iA12 * FLT_SCALE;
                A22 = iA22 * FLT_SCALE;

                float D = A11 * A22 - A12 * A12;
                float minEig = (A22 + A11 - std::sqrt((A11 - A22) * (A11 - A22) + 4.f * A12 * A12)) / (2 * winSize.width * winSize.height);

                if (minEig < minEigThreshold || D < FLT_EPSILON)
                {
                    if (level == 0)
                        status[i] = false;
                    continue;
                }

                D = 1.f / D;

                //Compute optical flow
                Point2f delta((float)((A12 * b2 - A22 * b1) * D),
                              (float)((A12 * b1 - A11 * b2) * D));

                nextPt += delta;
                nextPts[i].pt = nextPt + halfWin;

                if (nextPts[i].pt.x < (float)(halfWin.x + 1) || nextPts[i].pt.x >= (float)(J.cols - 1 - halfWin.x) ||
                    nextPts[i].pt.y < (float)(halfWin.y + 1) || nextPts[i].pt.y >= (float)(J.rows - 1 - halfWin.y))
                {
                    if (level == 0)
                        status[i] = false;
                    break;
                }

                if (delta.ddot(delta) <= epsilon)
                    break;

                if (j > 0 && std::abs(delta.x + prevDelta.x) < 0.01 &&
                    std::abs(delta.y + prevDelta.y) < 0.01)
                {
                    nextPts[i].pt -= delta * 0.5f;
                    break;
                }
                prevDelta = delta;
            }
        }
    }

    int toReturn = 0;

    vector<cv::Mat> vOutWins;
    vOutWins.resize(status.size());
    const cv::Mat J = newPyr[0].clone();
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            float meanJ = 0.f, meanJ2 = 0.f;
            float meanI = vMeanI[0][i], meanI2 = vMeanI2[0][i];

            cv::Mat win = cv::Mat(winSize, CV_16S);
            vOutWins[i] = cv::Mat(winSize, CV_8U);
            cv::Point2f nextPt = nextPts[i].pt - halfWin;
            cv::Point2i inextPt(cvFloor(nextPt.x), cvFloor(nextPt.y));

            const int W_BITS = 14, W_BITS1 = 14;
            const float FLT_SCALE = 1.f / (1 << 20);
            float aJ = nextPt.x - inextPt.x;
            float bJ = nextPt.y - inextPt.y;
            int jw00 = cvRound((1.f - aJ) * (1.f - bJ) * (1 << W_BITS));
            int jw01 = cvRound(aJ * (1.f - bJ) * (1 << W_BITS));
            int jw10 = cvRound((1.f - aJ) * bJ * (1 << W_BITS));
            int jw11 = (1 << W_BITS) - jw00 - jw01 - jw10;

            int stepJ = (int)(J.step / J.elemSize1());
            int cn = J.channels();

            int x, y;

            for (y = 0; y < winSize.height; y++)
            {
                //Get pointers to the images
                const uchar *src = J.ptr() + (y + inextPt.y) * stepJ + inextPt.x * cn;

                x = 0;

                for (; x < winSize.width; x++)
                {
                    //Get sub pixel values from images
                    int jval = CV_DESCALE(src[x] * jw00 + src[x + cn] * jw01 +
                                              src[x + stepJ] * jw10 + src[x + stepJ + cn] * jw11,
                                          W_BITS1 - 5);

                    //Compute accum values for later gain and bias computation
                    meanJ += (float)jval;
                    meanJ2 += (float)(jval * jval);

                    win.at<short>(y, x) = (short)(jval);
                }
            }
            //Compute means for later gain and bias computation
            meanJ = (meanJ * FLT_SCALE) / winSize.area();
            meanJ2 = (meanJ2 * FLT_SCALE) / winSize.area();

            //Compute alpha and beta
            float alpha = sqrt(meanI2 / meanJ2);
            float beta = meanI - alpha * meanJ;

            //Correct illumination
            cv::Mat corrected = alpha * win - beta;
            corrected /= 32;
            corrected.convertTo(vOutWins[i], CV_8U);
        }
        else
        {
            vOutWins[i] = cv::Mat(winSize, CV_8U, cv::Scalar(0));
        }
    }

    vHessian.resize(status.size());

    //Check outliers with SSIM
    const float C1 = (0.01 * 255) * (0.01 * 255), C2 = (0.03 * 255) * (0.03 * 255);
    const float N_inv = 1.f / (float)winSize.area(), N_inv_1 = 1.f / (float)(winSize.area() - 1);
    for (size_t i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            cv::Mat refWin = Iref[0][i].clone() / 32;
            cv::Mat currWin = vOutWins[i];

            //Compute means (x -> ref, y -> curr)
            float mu_x = 0.f, mu_y = 0.f;
            float sigma_x = 0.f, sigma_y = 0.f, sigma_xy = 0.f;

            for (int y = 0; y < winSize.height; y++)
            {
                const short *pRefWin = refWin.ptr<short>(y);
                const uchar *pCurrWin = currWin.ptr(y);

                for (int x = 0; x < winSize.width; x++)
                {
                    mu_x += (float)pRefWin[x];
                    mu_y += (float)pCurrWin[x];
                }
            }

            mu_x *= N_inv;
            mu_y *= N_inv;

            refWin.convertTo(refWin, CV_32F);
            currWin.convertTo(currWin, CV_32F);

            //Compute covs
            cv::Mat x_norm = refWin - mu_x;
            cv::Mat y_norm = currWin - mu_y;

            sigma_x = sqrtf(x_norm.dot(x_norm) * N_inv_1);
            sigma_y = sqrtf(y_norm.dot(y_norm) * N_inv_1);
            sigma_xy = x_norm.dot(y_norm) * N_inv_1;

            float SSIM = ((2.f * mu_x * mu_y + C1) * (2.f * sigma_xy + C2)) /
                         ((mu_x * mu_x + mu_y * mu_y + C1) * (sigma_x * sigma_x + sigma_y * sigma_y + C2));

            cv::Mat hessian = jac_[i].t() * jac_[i];
            hessian.copyTo(vHessian[i]);

            if (SSIM < minSSIM)
            {
                status[i] = false;
            }
            else
            {
                toReturn++;
            }
        }
    }
    return toReturn;
}

int LucasKanadeTracker::interpolate(int &a, int &b, int &c, int &d, int w, int x, int y, int z, int WBITS)
{
    __m128i abcd = _mm_set_epi32(d, c, b, a);
    __m128i wxyz = _mm_set_epi32(z, y, x, w);

    __m128i product = _mm_mullo_epi32(wxyz, abcd);

    product = _mm_hadd_epi32(product, product);
    product = _mm_hadd_epi32(product, product);

    int sum = _mm_extract_epi32(product, 0);
    return CV_DESCALE(sum, WBITS);
}

int LucasKanadeTracker::PRE_Track_VEC(Mat &newIm, std::vector<KeyPoint> &nextPts, vector<bool> &status, const bool bInitialFlow,
                                      const float minSSIM, std::vector<float> &vSSIM, std::vector<cv::Mat> &vOutWins)
{
    //Dimensions of half of the window
    Point2f halfWin((winSize.width - 1) * 0.5f, (winSize.height - 1) * 0.5f);

    //Compute pyramid images
    vector<Mat> newPyr;
    cv::buildOpticalFlowPyramid(newIm, newPyr, winSize, maxLevel);

    const int rows = winSize.height;
    const int cols = winSize.width;

    //Start Lucas-Kanade optical flow algorithm
    //First iterate over pyramid levels
    for (int level = maxLevel; level >= 0; level--)
    {
        //Get images and gradients
        const Mat I = refPyr[level * 2];
        const Mat J = newPyr[level * 2];
        const Mat derivI = refPyr[level * 2 + 1];
        const Mat derivJ = newPyr[level * 2 + 1];

        //Buffer for fast memory access
        int j, cn = I.channels(), cn2 = cn * 2; //cn should be 1 therefor cn2 should be 2
        AutoBuffer<short> _buf(winSize.area() * (cn + cn2) * 2);
        int derivDepth = DataType<short>::depth;

        //Integration window buffers
        Mat IWinBuf(winSize, CV_MAKETYPE(derivDepth, cn), (*_buf));
        Mat JWinBuf(winSize, CV_MAKETYPE(derivDepth, cn), (*_buf) + winSize.area());
        Mat derivIWinBuf(winSize, CV_MAKETYPE(derivDepth, cn2), (*_buf) + 2 * winSize.area());
        Mat derivJWinBuf(winSize, CV_MAKETYPE(derivDepth, cn2), (*_buf) + 4 * winSize.area());

        //Steps for matrix indexing
        int dstep = (int)(derivI.step / derivI.elemSize1());
        int stepJ = (int)(J.step / J.elemSize1());

        //Track each point at the current pyramid level
        for (int i = 0; i < prevPts.size(); i++)
        {
            if (!status[i])
                continue;

            //Compute image coordinates in the reference image at the current level
            Point2f prevPt = prevPts[i].pt * (float)(1. / (1 << level));
            //Compute image coordinates in the current frame at the current level
            Point2f nextPt;
            if (level == maxLevel)
            {
                //if (level == maxLevel) {
                if (bInitialFlow)
                {
                    nextPt = nextPts[i].pt * (float)(1. / (1 << level));
                }
                else
                {
                    nextPt = prevPt;
                }
            }
            else
            {
                nextPt = nextPts[i].pt * 2.f;
            }

            nextPts[i].pt = nextPt;

            //Check that previous point and next point is inside of the
            //image boundaries
            Point2i iprevPt, inextPt;
            prevPt -= halfWin;
            iprevPt.x = cvFloor(prevPt.x);
            iprevPt.y = cvFloor(prevPt.y);

            if (iprevPt.x < -winSize.width || iprevPt.x >= derivI.cols ||
                iprevPt.y < -winSize.height || iprevPt.y >= derivI.rows)
            {
                if (level == 0)
                    status[i] = false;

                continue;
            }

            //Compute weighs for sub pixel computation
            const int W_BITS = 14, W_BITS1 = 14;
            const float FLT_SCALE = 1.f / (1 << 20);

            //Compute sumI, sumI2, meanI, meanI2, Iwin, IdWin
            float meanI = 0.f, meanI2 = 0.f;

            int x, y;
            //Compute means for later gain and bias computation
            meanI = vMeanI[level][i];
            meanI2 = vMeanI2[level][i];

            IWinBuf = Iref[level][i].clone();
            derivIWinBuf = Idref[level][i].clone();

            //Optical flow loop
            Point2f prevDelta;
            nextPt -= halfWin;
            for (j = 0; j < maxIters; j++)
            {
                //Compute weighs for sub pixel computation
                inextPt.x = cvFloor(nextPt.x);
                inextPt.y = cvFloor(nextPt.y);

                //Check that the point is inside the image
                if (inextPt.x < -winSize.width || inextPt.x >= J.cols ||
                    inextPt.y < -winSize.height || inextPt.y >= J.rows)
                {
                    if (level == 0)
                        status[i] = false;
                    break;
                }

                float aJ = nextPt.x - inextPt.x;
                float bJ = nextPt.y - inextPt.y;
                int jw00 = cvRound((1.f - aJ) * (1.f - bJ) * (1 << W_BITS));
                int jw01 = cvRound(aJ * (1.f - bJ) * (1 << W_BITS));
                int jw10 = cvRound((1.f - aJ) * bJ * (1 << W_BITS));
                int jw11 = (1 << W_BITS) - jw00 - jw01 - jw10;

                //Compute alpha and beta for gain and bias
                //Compute sumI, sumI2, meanI, meanI2, Iwin, IdWin
                float meanJ = 0.f, meanJ2 = 0.f;

                for (y = 0; y < rows; y++)
                {
                    //Get pointers to the images
                    const uchar *src = J.ptr() + (y + inextPt.y) * stepJ + inextPt.x * cn;
                    const short *dsrc = derivJ.ptr<short>() + (y + inextPt.y) * dstep + inextPt.x * cn2;

                    //Get pointers to the window buffers
                    short *Jptr = JWinBuf.ptr<short>(y);
                    short *dJptr = derivJWinBuf.ptr<short>(y);

                    x = 0;

                    for (; x < cols * cn; x++, dsrc += 2, dJptr += 2)
                    {
                        //Get sub pixel values from images
                        int jval = CV_DESCALE(src[x] * jw00 + src[x + cn] * jw01 +
                                                  src[x + stepJ] * jw10 + src[x + stepJ + cn] * jw11,
                                              W_BITS1 - 5);
                        int jxval = CV_DESCALE(dsrc[0] * jw00 + dsrc[cn2] * jw01 +
                                                   dsrc[dstep] * jw10 + dsrc[dstep + cn2] * jw11,
                                               W_BITS1);
                        int jyval = CV_DESCALE(dsrc[1] * jw00 + dsrc[cn2 + 1] * jw01 + dsrc[dstep + 1] * jw10 +
                                                   dsrc[dstep + cn2 + 1] * jw11,
                                               W_BITS1);

                        /*int jval  = interpolate(jw00,jw01,jw10,jw11,src[x],src[x + cn],src[x + stepJ],src[x + stepJ + cn], W_BITS1 - 5);
                        int jxval = interpolate(jw00,jw01,jw10,jw11,dsrc[0],dsrc[cn2],dsrc[dstep],dsrc[dstep + cn2],W_BITS1);
                        int jyval = interpolate(jw00,jw01,jw10,jw11,dsrc[1],dsrc[cn2+1],dsrc[dstep+1],dsrc[dstep + cn2+1],W_BITS1);*/

                        //cout << jvalv << "(" << jvalv << ") --- " << jxvalV << "(" << jxval << ") --- " << jyvalV << "(" << jyval << ")" << endl;

                        //Store values to the window buffers
                        Jptr[x] = (short)jval;
                        dJptr[0] = (short)jxval;
                        dJptr[1] = (short)jyval;

                        //Compute accum values for later gain and bias computation
                        meanJ += (float)jval;
                        meanJ2 += (float)(jval * jval);
                    }
                }

                //Compute means for later gain and bias computation
                meanJ = (meanJ * FLT_SCALE) / winSize.area();
                meanJ2 = (meanJ2 * FLT_SCALE) / winSize.area();

                //Compute alpha and beta
                float alpha = sqrt(meanI2 / meanJ2);
                float beta = meanI - alpha * meanJ;

                //Compute image gradient insensitive to ilumination changes
                float ib1 = 0, ib2 = 0;
                float b1, b2;
                float iA11 = 0, iA12 = 0, iA22 = 0;
                float A11, A12, A22;

                for (y = 0; y < rows; y++)
                {
                    //Get pointers to the buffers
                    const short *Iptr = IWinBuf.ptr<short>(y);
                    const short *Jptr = JWinBuf.ptr<short>(y);
                    const short *dIptr = derivIWinBuf.ptr<short>(y);
                    const short *dJptr = derivJWinBuf.ptr<short>(y);

                    x = 0;
                    for (; x < cols * cn; x++, dIptr += 2, dJptr += 2)
                    {
                        int diff = Jptr[x] * alpha - Iptr[x] - beta;
                        float dx = (float)(dIptr[0] + dJptr[0] * alpha);
                        float dy = (float)(dIptr[1] + dJptr[1] * alpha);

                        ib1 += (float)(diff * dx);
                        ib2 += (float)(diff * dy);

                        iA11 += (float)(dx * dx);
                        iA22 += (float)(dy * dy);
                        iA12 += (float)(dx * dy);
                    }
                }
                b1 = ib1 * FLT_SCALE;
                b2 = ib2 * FLT_SCALE;

                //Compute spatial gradient matrix
                A11 = iA11 * FLT_SCALE;
                A12 = iA12 * FLT_SCALE;
                A22 = iA22 * FLT_SCALE;

                float D = A11 * A22 - A12 * A12;
                float minEig = (A22 + A11 - std::sqrt((A11 - A22) * (A11 - A22) + 4.f * A12 * A12)) / (2 * winSize.width * winSize.height);

                if (minEig < minEigThreshold || D < FLT_EPSILON)
                {
                    if (level == 0)
                        status[i] = false;
                    continue;
                }

                D = 1.f / D;

                //Compute optical flow
                Point2f delta((float)((A12 * b2 - A22 * b1) * D),
                              (float)((A12 * b1 - A11 * b2) * D));

                nextPt += delta;
                nextPts[i].pt = nextPt + halfWin;

                if (nextPts[i].pt.x < halfWin.x + 1 || nextPts[i].pt.x >= J.cols - 1 - halfWin.x ||
                    nextPts[i].pt.y < halfWin.y + 1 || nextPts[i].pt.y >= J.rows - 1 - halfWin.y)
                {
                    if (level == 0)
                        status[i] = false;
                    break;
                }

                if (delta.ddot(delta) <= epsilon)
                    break;

                if (j > 0 && std::abs(delta.x + prevDelta.x) < 0.01 &&
                    std::abs(delta.y + prevDelta.y) < 0.01)
                {
                    nextPts[i].pt -= delta * 0.5f;
                    break;
                }
                prevDelta = delta;
            }
        }
    }

    int toReturn = 0;

    vOutWins.resize(status.size());
    vSSIM.resize(status.size());
    const cv::Mat J = newPyr[0].clone();
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            float meanJ = 0.f, meanJ2 = 0.f;
            float meanI = vMeanI[0][i], meanI2 = vMeanI2[0][i];

            cv::Mat win = cv::Mat(winSize, CV_16S);
            vOutWins[i] = cv::Mat(winSize, CV_8U);
            cv::Point2f nextPt = nextPts[i].pt - halfWin;
            cv::Point2i inextPt(cvFloor(nextPt.x), cvFloor(nextPt.y));

            const int W_BITS = 14, W_BITS1 = 14;
            const float FLT_SCALE = 1.f / (1 << 20);
            float aJ = nextPt.x - inextPt.x;
            float bJ = nextPt.y - inextPt.y;
            int jw00 = cvRound((1.f - aJ) * (1.f - bJ) * (1 << W_BITS));
            int jw01 = cvRound(aJ * (1.f - bJ) * (1 << W_BITS));
            int jw10 = cvRound((1.f - aJ) * bJ * (1 << W_BITS));
            int jw11 = (1 << W_BITS) - jw00 - jw01 - jw10;

            int stepJ = (int)(J.step / J.elemSize1());
            int cn = J.channels();

            int x, y;

            for (y = 0; y < rows; y++)
            {
                //Get pointers to the images
                const uchar *src = J.ptr() + (y + inextPt.y) * stepJ + inextPt.x * cn;

                x = 0;

                for (; x < cols * cn; x++)
                {
                    //Get sub pixel values from images
                    //int jval = CV_DESCALE(src[x] * jw00 + src[x + cn] * jw01 +
                    //                      src[x + stepJ] * jw10 + src[x + stepJ + cn] * jw11, W_BITS1 - 5);

                    int jval = interpolate(jw00, jw01, jw10, jw11, src[x], src[x + cn], src[x + stepJ], src[x + stepJ + cn], W_BITS1 - 5);

                    //Compute accum values for later gain and bias computation
                    meanJ += (float)jval;
                    meanJ2 += (float)(jval * jval);

                    win.at<short>(y, x) = (short)(jval);
                }
            }
            //Compute means for later gain and bias computation
            meanJ = (meanJ * FLT_SCALE) / winSize.area();
            meanJ2 = (meanJ2 * FLT_SCALE) / winSize.area();

            //Compute alpha and beta
            float alpha = sqrt(meanI2 / meanJ2);
            float beta = meanI - alpha * meanJ;

            //Correct illumination
            cv::Mat corrected = alpha * win - beta;
            corrected /= 32;
            corrected.convertTo(vOutWins[i], CV_8U);
        }
        else
        {
            vOutWins[i] = cv::Mat(winSize, CV_8U, cv::Scalar(0));
        }
    }

    //Check outliers with SSIM
    const float C1 = (0.01 * 255) * (0.01 * 255), C2 = (0.03 * 255) * (0.03 * 255);
    const float N_inv = 1.f / (float)winSize.area(), N_inv_1 = 1.f / (float)(winSize.area() - 1);
    for (size_t i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            cv::Mat refWin = Iref[0][i].clone() / 32;
            cv::Mat currWin = vOutWins[i];

            //Compute means (x -> ref, y -> curr)
            float mu_x = 0.f, mu_y = 0.f;
            float sigma_x = 0.f, sigma_y = 0.f, sigma_xy = 0.f;

            for (int y = 0; y < rows; y++)
            {
                const short *pRefWin = refWin.ptr<short>(y);
                const uchar *pCurrWin = currWin.ptr(y);

                /*for(int x = 0; x < cols; x++){
                    mu_x += (float) pRefWin[x];
                    mu_y += (float) pCurrWin[x];
                }*/

                for (int x = 0; x < cols; x++)
                    mu_y += (float)pCurrWin[x];
            }

            mu_x *= N_inv;
            mu_y *= N_inv;

            refWin.convertTo(refWin, CV_32F);
            currWin.convertTo(currWin, CV_32F);

            //Compute covs
            cv::Mat x_norm = refWin - mu_x;
            cv::Mat y_norm = currWin - mu_y;

            sigma_x = sqrtf(x_norm.dot(x_norm) * N_inv_1);
            sigma_y = sqrtf(y_norm.dot(y_norm) * N_inv_1);
            sigma_xy = x_norm.dot(y_norm) * N_inv_1;

            float SSIM = ((2.f * mu_x * mu_y + C1) * (2.f * sigma_xy + C2)) /
                         ((mu_x * mu_x + mu_y * mu_y + C1) * (sigma_x * sigma_x + sigma_y * sigma_y + C2));

            vSSIM[i] = SSIM;
            if (SSIM < minSSIM)
            {
                status[i] = false;
            }
            else
            {
                toReturn++;
            }
        }
    }
    return toReturn;
}

void LucasKanadeTracker::AddFromKeyFrame(ORB_SLAM2::KeyFrame *pKF, std::vector<cv::KeyPoint> &vKeys,
                                         vector<ORB_SLAM2::MapPoint *> &vMPs)
{
    vector<ORB_SLAM2::MapPoint *> vKfMps = pKF->GetMapPointMatches();

    if (vKfMps.empty())
        return;

    std::set<ORB_SLAM2::MapPoint *> setklt;
    vector<ORB_SLAM2::MapPoint *> vectorklt;
    for (ORB_SLAM2::MapPoint *pMP : vMPs)
    {
        if (pMP)
        {
            setklt.insert(pMP);
            vectorklt.push_back(pMP);
        }
    }

    cout << "[AddFromKeyFrame-0]: " << vectorklt.size() << " --- " << setklt.size() << endl;

    //First update already tracked MapPoints associated to KeyPoints
    /*for (int i = 0; i < pKF->nKLTfeatures; i++) {
        vMPs[i] = vKfMps[i];
        if (vKfMps[i])
            vKfMps[i]->trackedByKLT = true;
    }*/

    std::set<ORB_SLAM2::MapPoint *> setklt1;
    vector<ORB_SLAM2::MapPoint *> vectorklt1;
    for (ORB_SLAM2::MapPoint *pMP : vMPs)
    {
        if (pMP)
        {
            setklt1.insert(pMP);
            vectorklt1.push_back(pMP);
        }
    }

    cout << "[AddFromKeyFrame-1]: " << vectorklt1.size() << " --- " << setklt1.size() << endl;

    //Then add new MapPoints triangulated at LocalMapping
    for (int i = pKF->nKLTfeatures; i < pKF->N; i++)
    {
        ORB_SLAM2::MapPoint *pMP = vKfMps[i];
        if (pMP && !pMP->trackedByKLT)
        {
            pMP->trackedByKLT = true;

            pMP->KLT_ComputeMatrices(pMP->mObs);

            vKeys.push_back(pKF->mvKeys[i]);
            vMPs.push_back(vKfMps[i]);
            prevPts.push_back(pKF->mvKeys[i]);

            //Add image information
            for (int level = 0; level <= maxLevel; level++)
            {
                Iref[level].push_back(pMP->mvPatch[level].clone());
                Idref[level].push_back(pMP->mvGrad[level].clone());
                vMeanI[level].push_back(pMP->mvMean[level]);
                vMeanI2[level].push_back(pMP->mvMean2[level]);
            }
        }
    }

    std::set<ORB_SLAM2::MapPoint *> setklt2;
    vector<ORB_SLAM2::MapPoint *> vectorklt2;
    for (ORB_SLAM2::MapPoint *pMP : vMPs)
    {
        if (pMP)
        {
            setklt2.insert(pMP);
            vectorklt2.push_back(pMP);
        }
    }

    cout << "[AddFromKeyFrame-2]: " << vectorklt2.size() << " --- " << setklt2.size() << endl;
}

int LucasKanadeTracker::TrackWithInfoWithHH(cv::Mat &newIm, std::vector<cv::KeyPoint> &nextPts,
                                            const std::vector<cv::KeyPoint> &prevPts,
                                            std::vector<bool> &status,
                                            const std::vector<std::vector<cv::Mat>> vPatches,
                                            const std::vector<std::vector<cv::Mat>> vGrad,
                                            const std::vector<std::vector<float>> vMean,
                                            const std::vector<std::vector<float>> vMean2,
                                            std::vector<cv::Mat> &vH, const float minSSIM,
                                            std::vector<cv::Mat>& vHessian)
{
    auto startPos = nextPts;
    //Set status of all the points to true
    status = vector<bool>(nextPts.size(), true);

    //covariance
    vector<cv::Mat> jac_(nextPts.size());
    for(size_t i = 0; i < jac_.size(); i++){
        jac_[i] = cv::Mat(winSize.area(),2,CV_32F);
    }

    //Dimensions of half of the window
    Point2f halfWin((winSize.width - 1) * 0.5f, (winSize.height - 1) * 0.5f);

    //Compute new pyramid image
    vector<Mat> newPyr;
    cv::buildOpticalFlowPyramid(newIm, newPyr, winSize, maxLevel);

    std::vector<std::vector<float>> vRelCordsX, vRelCordsY;
    ComputeRelativeCoordinates(prevPts, vH, vRelCordsX, vRelCordsY);

    //Start Lucas-Kanade optical flow algorithm
    //First iterate over pyramid levels
    for (int level = maxLevel; level >= 0; level--)
    {
        //Get current image and gradient
        const Mat J = newPyr[level * 2];
        const Mat derivJ = newPyr[level * 2 + 1];

        //Buffer for fast memory access
        int j, cn = J.channels(), cn2 = cn * 2; //cn should be 1 therefor cn2 should be 2
        AutoBuffer<short> _buf(winSize.area() * (cn + cn2) * 2);
        int derivDepth = DataType<short>::depth;

        //Integration window buffers
        Mat IWinBuf(winSize, CV_MAKETYPE(derivDepth, cn), (*_buf));
        Mat JWinBuf(winSize, CV_MAKETYPE(derivDepth, cn), (*_buf) + winSize.area());
        Mat derivIWinBuf(winSize, CV_MAKETYPE(derivDepth, cn2), (*_buf) + 2 * winSize.area());
        Mat derivJWinBuf(winSize, CV_MAKETYPE(derivDepth, cn2), (*_buf) + 4 * winSize.area());

        //Steps for matrix indexing
        int dstep = (int)(derivJ.step / derivJ.elemSize1());
        int stepJ = (int)(J.step / J.elemSize1());

        //Track each point at the current pyramid level
        for (int i = 0; i < prevPts.size(); i++)
        {

            Point2f nextPt;
            //Compute the whole window coordinates
            if (level == maxLevel)
            {
                nextPt = nextPts[i].pt * (float)(1. / (1 << level));
            }
            else
            {
                nextPt = nextPts[i].pt * 2.f;
            }

            //Compute weighs for sub pixel computation
            const int W_BITS = 14, W_BITS1 = 14;
            const float FLT_SCALE = 1.f / (1 << 20);

            //Compute sumI, sumI2, meanI, meanI2, Iwin, IdWin
            float meanI, meanI2;

            int x, y;
            //Compute means for later gain and bias computation
            meanI = vMean[i][level];
            meanI2 = vMean2[i][level];

            IWinBuf = vPatches[i][level].clone();
            derivIWinBuf = vGrad[i][level].clone();

            //Optical flow loop
            Point2f prevDelta;

            bool out = false;

            //Get relative coordinates
            std::vector<float> vRelX = vRelCordsX[i], vRelY = vRelCordsY[i];

            for (j = 0; j < maxIters; j++)
            {
                if (out)
                    break;
                //Point2f pRef = nextPt;
                //Compute alpha and beta for gain and bias
                //Compute sumI, sumI2, meanI, meanI2, Iwin, IdWin
                int jw00, jw01, jw10, jw11;
                float meanJ = 0.f, meanJ2 = 0.f;

                Point2i inextPt;

                inextPt.x = cvFloor(nextPt.x - halfWin.x);
                inextPt.y = cvFloor(nextPt.y - halfWin.y);

                if (inextPt.x < -winSize.width || inextPt.x >= J.cols ||
                    inextPt.y < -winSize.height || inextPt.y >= J.rows)
                {
                    if (level == 0)
                        status[i] = false;
                    break;
                }

                int xRi, yRi;

                int z = 0;
                for (y = 0; y < winSize.height; y++)
                {
                    if (out)
                        break;

                    short *Jptr = JWinBuf.ptr<short>(y);
                    short *dJptr = derivJWinBuf.ptr<short>(y);

                    x = 0;

                    for (; x < winSize.width * cn; x++, dJptr += 2)
                    {
                        Point2f p = cv::Point2f(nextPt.x + vRelX[z], nextPt.y + vRelY[z]);

                        if (p.x < -winSize.width || p.x >= derivJ.cols ||
                            p.y < -winSize.height || p.y >= derivJ.rows)
                        {
                            out = true;
                            break;
                        }

                        //Compute weighs for sub pixel computation
                        SubPixWeights(p, jw00, jw01, jw10, jw11, xRi, yRi);

                        //Get pointers
                        const uchar *srcRot = J.ptr() + yRi * stepJ + xRi * cn;
                        const short *dsrcRot = derivJ.ptr<short>() + yRi * dstep + xRi * cn2;

                        //Get sub pixel values from images
                        int jxval = CV_DESCALE(dsrcRot[0] * jw00 + dsrcRot[cn2] * jw01 +
                                                   dsrcRot[dstep] * jw10 + dsrcRot[dstep + cn2] * jw11,
                                               W_BITS1);
                        int jval = CV_DESCALE(srcRot[0] * jw00 + srcRot[cn] * jw01 +
                                                  srcRot[stepJ] * jw10 + srcRot[stepJ + cn] * jw11,
                                              W_BITS1 - 5);
                        int jyval = CV_DESCALE(dsrcRot[1] * jw00 + dsrcRot[cn2 + 1] * jw01 + dsrcRot[dstep + 1] * jw10 +
                                                   dsrcRot[dstep + cn2 + 1] * jw11,
                                               W_BITS1);

                        //Store values to the window buffers
                        Jptr[x] = (short)jval;
                        dJptr[0] = (short)jxval;
                        dJptr[1] = (short)jyval;

                        //Compute accum values for later gain and bias computation
                        meanJ += (float)jval;
                        meanJ2 += (float)(jval * jval);

                        z++;
                    }
                }

                if (out)
                {
                    if (level == 0)
                        status[i] = false;
                    break;
                }

                //Compute means for later gain and bias computation
                meanJ = (meanJ * FLT_SCALE) / winSize.area();
                meanJ2 = (meanJ2 * FLT_SCALE) / winSize.area();

                //Compute alpha and beta
                float alpha = sqrt(meanI2 / meanJ2);
                float beta = meanI - alpha * meanJ;

                //Compute image gradient insensitive to ilumination changes
                float ib1 = 0, ib2 = 0;
                float b1, b2;
                float iA11 = 0, iA12 = 0, iA22 = 0;
                float A11, A12, A22;

                int rr = 0;

                for (y = 0; y < winSize.height; y++)
                {
                    //Get pointers to the buffers
                    const short *Iptr = IWinBuf.ptr<short>(y);
                    const short *Jptr = JWinBuf.ptr<short>(y);
                    const short *dIptr = derivIWinBuf.ptr<short>(y);
                    const short *dJptr = derivJWinBuf.ptr<short>(y);

                    x = 0;
                    for (; x < winSize.width * cn; x++, dIptr += 2, dJptr += 2)
                    {
                        int diff = Jptr[x] * alpha - Iptr[x] - beta;
                        float dx = (float)(dIptr[0] + dJptr[0] * alpha);
                        float dy = (float)(dIptr[1] + dJptr[1] * alpha);

                        ib1 += (float)(diff * dx);
                        ib2 += (float)(diff * dy);

                        iA11 += (float)(dx * dx);
                        iA22 += (float)(dy * dy);
                        iA12 += (float)(dx * dy);

                        jac_[i].at<float>(rr,0) = (float) (diff * dx);
                        jac_[i].at<float>(rr,1) = (float) (diff * dy);
                        rr++;
                    }
                }
                b1 = ib1 * FLT_SCALE;
                b2 = ib2 * FLT_SCALE;

                jac_[i] *= FLT_SCALE;

                //Compute spatial gradient matrix
                A11 = iA11 * FLT_SCALE;
                A12 = iA12 * FLT_SCALE;
                A22 = iA22 * FLT_SCALE;

                float D = A11 * A22 - A12 * A12;
                float minEig = (A22 + A11 - std::sqrt((A11 - A22) * (A11 - A22) + 4.f * A12 * A12)) / (2 * winSize.width * winSize.height);

                if (minEig < minEigThreshold || D < FLT_EPSILON)
                {
                    if (level == 0)
                        status[i] = false;
                    continue;
                }

                D = 1.f / D;

                //Compute optical flow
                Point2f delta((float)((A12 * b2 - A22 * b1) * D),
                              (float)((A12 * b1 - A11 * b2) * D));

                nextPt += delta;
                nextPts[i].pt = nextPt; // + halfWin;

                if (delta.ddot(delta) <= epsilon)
                    break;

                if (j > 0 && std::abs(delta.x + prevDelta.x) < 0.01 &&
                    std::abs(delta.y + prevDelta.y) < 0.01)
                {
                    nextPts[i].pt -= delta * 0.5f;
                    break;
                }
                prevDelta = delta;
            }
        }
    }

    const Mat J = newPyr[0];
    int stepJ = (int)(J.step / J.elemSize1());
    int cn = J.channels();
    vector<cv::Mat> vOutWins;
    vOutWins.resize(status.size());

    for (int i = 0; i < status.size(); i++)
    {
        vOutWins[i] = cv::Mat(winSize, CV_8U);
        cv::Mat win = cv::Mat(winSize, CV_16S);

        float meanJ = 0.f, meanJ2 = 0.f;
        float meanI = vMean[i][0], meanI2 = vMean2[i][0];

        std::vector<float> vRelX = vRelCordsX[i], vRelY = vRelCordsY[i];
        cv::Point2f nextPt = nextPts[i].pt;

        if (nextPt.x != nextPt.x || nextPt.y != nextPt.y)
        {
            status[i] = false;
            continue;
        }
        int z = 0;
        const float FLT_SCALE = 1.f / (1 << 20);

        int jw00, jw01, jw10, jw11;
        int xRi, yRi;
        const int W_BITS = 14, W_BITS1 = 14;

        bool out = false;

        for (int y = 0; y < winSize.height; y++)
        {
            if (out)
                break;

            for (int x = 0; x < winSize.width * cn; x++)
            {
                Point2f p = cv::Point2f(nextPt.x + vRelX[z], nextPt.y + vRelY[z]);
                if (p.x < -winSize.height || p.x >= J.cols ||
                    p.y < -winSize.height || p.y >= J.rows)
                {
                    out = true;
                    break;
                }

                SubPixWeights(p, jw00, jw01, jw10, jw11, xRi, yRi);

                const uchar *srcRot = J.ptr() + yRi * stepJ + xRi * cn;

                int jval = CV_DESCALE(srcRot[0] * jw00 + srcRot[cn] * jw01 +
                                          srcRot[stepJ] * jw10 + srcRot[stepJ + cn] * jw11,
                                      W_BITS1 - 5);

                //Compute accum values for later gain and bias computation
                meanJ += (float)jval;
                meanJ2 += (float)(jval * jval);

                win.at<short>(y, x) = (short)(jval);

                z++;
            }
        }

        //Compute means for later gain and bias computation
        meanJ = (meanJ * FLT_SCALE) / winSize.area();
        meanJ2 = (meanJ2 * FLT_SCALE) / winSize.area();

        //Compute alpha and beta
        float alpha = sqrt(meanI2 / meanJ2);
        float beta = meanI - alpha * meanJ;

        //Correct illumination
        cv::Mat corrected = alpha * win - beta;
        corrected /= 32;
        corrected.convertTo(vOutWins[i], CV_8U);
    }

    vHessian.resize(status.size());

    //Check outliers with SSIM
    int toReturn = 0;
    const float C1 = (0.01 * 255) * (0.01 * 255), C2 = (0.03 * 255) * (0.03 * 255);
    const float N_inv = 1.f / (float)winSize.area(), N_inv_1 = 1.f / (float)(winSize.area() - 1);
    for (size_t i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            cv::Mat refWin = vPatches[i][0] / 32;
            cv::Mat currWin = vOutWins[i];

            //Compute means (x -> ref, y -> curr)
            float mu_x = 0.f, mu_y = 0.f;
            float sigma_x = 0.f, sigma_y = 0.f, sigma_xy = 0.f;

            for (int y = 0; y < winSize.height; y++)
            {
                const short *pRefWin = refWin.ptr<short>(y);
                const uchar *pCurrWin = currWin.ptr(y);

                for (int x = 0; x < winSize.width; x++)
                {
                    mu_x += (float)pRefWin[x];
                    mu_y += (float)pCurrWin[x];
                }
            }

            mu_x *= N_inv;
            mu_y *= N_inv;

            refWin.convertTo(refWin, CV_32F);
            currWin.convertTo(currWin, CV_32F);

            //Compute covs
            cv::Mat x_norm = refWin - mu_x;
            cv::Mat y_norm = currWin - mu_y;

            sigma_x = sqrtf(x_norm.dot(x_norm) * N_inv_1);
            sigma_y = sqrtf(y_norm.dot(y_norm) * N_inv_1);
            sigma_xy = x_norm.dot(y_norm) * N_inv_1;

            float SSIM = ((2.f * mu_x * mu_y + C1) * (2.f * sigma_xy + C2)) /
                         ((mu_x * mu_x + mu_y * mu_y + C1) * (sigma_x * sigma_x + sigma_y * sigma_y + C2));

            cv::Mat hessian = jac_[i].t() * jac_[i];
            hessian.copyTo(vHessian[i]);

            if (SSIM < minSSIM)
                status[i] = false;
            else
                toReturn++;
        }
    }

    return toReturn;
}

void LucasKanadeTracker::ComputeRelativeCoordinates(const std::vector<cv::KeyPoint> &refPts, std::vector<cv::Mat> &vH,
                                                    std::vector<std::vector<float>> &vRelCoorsX, std::vector<std::vector<float>> &vRelCoorsY)
{
    cv::Point2f halfWin((winSize.width - 1) * 0.5f, (winSize.height - 1) * 0.5f);

    vRelCoorsX = std::vector<std::vector<float>>(refPts.size(), std::vector<float>(winSize.area()));
    vRelCoorsY = std::vector<std::vector<float>>(refPts.size(), std::vector<float>(winSize.area()));

    for (int p = 0; p < refPts.size(); p++)
    {

        float *Hptr = vH[p].ptr<float>();
        cv::Point2f refPoint = refPts[p].pt - halfWin;
        cv::Point2f currPt;
        int i = 0;

        cv::Point2f centerH;
        Hwarp(Hptr, refPts[p].pt.x, refPts[p].pt.y, centerH.x, centerH.y);

        for (int y = 0; y < winSize.height; y++)
        {
            currPt.y = refPoint.y + y * 1.0f;
            for (int x = 0; x < winSize.width; x++)
            {
                currPt.x = refPoint.x + x * 1.0f;

                Hwarp(Hptr, currPt.x, currPt.y, vRelCoorsX[p][i], vRelCoorsY[p][i]);
                vRelCoorsX[p][i] -= centerH.x;
                vRelCoorsY[p][i] -= centerH.y;

                i++;
            }
        }
    }
}

inline void LucasKanadeTracker::Hwarp(float *pH, const float x, const float y, float &xR, float &yR)
{
    float h = 1.0f / (x * pH[6] + y * pH[7] + pH[8]);
    xR = (x * pH[0] + y * pH[1] + pH[2]) * h;
    yR = (x * pH[3] + y * pH[4] + pH[5]) * h;
}

inline void LucasKanadeTracker::SubPixWeights(cv::Point2f p, int &w00, int &w01, int &w10, int &w11, int &xRi, int &yRi)
{
    const int W_BITS = 14;

    //Compute weighs for sub pixel computation
    Point2f ip(cvFloor(p.x), cvFloor(p.y));

    xRi = ip.x;
    yRi = ip.y;

    float aJ = p.x - ip.x;
    float bJ = p.y - ip.y;
    w00 = cvRound((1.f - aJ) * (1.f - bJ) * (1 << W_BITS));
    w01 = cvRound(aJ * (1.f - bJ) * (1 << W_BITS));
    w10 = cvRound((1.f - aJ) * bJ * (1 << W_BITS));
    w11 = (1 << W_BITS) - w00 - w01 - w10;
}

void LucasKanadeTracker::AddPointsFromMapPoints(const vector<ORB_SLAM2::MapPoint *> &vpNewMPs,
                                                const std::vector<cv::KeyPoint> &vNewKeys, const cv::Mat &im,
                                                vector<ORB_SLAM2::MapPoint *> &vpUpdatedMPs,
                                                std::vector<cv::KeyPoint> &vUpdatedKeys)
{
    vector<cv::Mat> pyr;
    cv::buildOpticalFlowPyramid(im, pyr, winSize, maxLevel);

    Point2f halfWin((winSize.width - 1) * 0.5f, (winSize.height - 1) * 0.5f);

    for (int level = 0; level <= maxLevel; level++)
    {
        //Get images form the pyramid
        const Mat I = pyr[level * 2];
        const Mat derivI = pyr[level * 2 + 1];

        //Steps for matrix indexing
        int dstep = (int)(derivI.step / derivI.elemSize1());
        int stepI = (int)(I.step / I.elemSize1());

        //Buffer for fast memory access
        int cn = I.channels(), cn2 = cn * 2; //cn should be 1 therefor cn2 should be 2
        AutoBuffer<short> _buf(winSize.area() * (cn + cn2));
        int derivDepth = DataType<short>::depth;

        //Integration window buffers
        Mat IWinBuf(winSize, CV_MAKETYPE(derivDepth, cn), (*_buf));
        Mat derivIWinBuf(winSize, CV_MAKETYPE(derivDepth, cn2), (*_buf) + winSize.area() * cn);

        for (int i = 0; i < vpNewMPs.size(); i++)
        {
            if (vpNewMPs[i] && !vpNewMPs[i]->trackedByKLT)
            {
                //Compute image coordinates in the reference image at the current level
                Point2f point = vNewKeys[i].pt / (1 << level);

                Point2i ipoint;
                point -= halfWin;
                ipoint.x = cvFloor(point.x);
                ipoint.y = cvFloor(point.y);

                //Compute weighs for sub pixel computation
                float a = point.x - ipoint.x;
                float b = point.y - ipoint.y;
                const int W_BITS = 14, W_BITS1 = 14;
                const float FLT_SCALE = 1.f / (1 << 20);
                int iw00 = cvRound((1.f - a) * (1.f - b) * (1 << W_BITS));
                int iw01 = cvRound(a * (1.f - b) * (1 << W_BITS));
                int iw10 = cvRound((1.f - a) * b * (1 << W_BITS));
                int iw11 = (1 << W_BITS) - iw00 - iw01 - iw10;

                //Compute sumI, sumI2, meanI, meanI2, Iwin, IdWin
                float meanI = 0.f, meanI2 = 0.f;

                int x, y;
                for (y = 0; y < winSize.height; y++)
                {
                    //Get pointers to the images
                    const uchar *src = I.ptr() + (y + ipoint.y) * stepI + ipoint.x;
                    const short *dsrc = derivI.ptr<short>() + (y + ipoint.y) * dstep + ipoint.x * 2;

                    //Get pointers to the window buffers
                    short *Iptr = IWinBuf.ptr<short>(y);
                    short *dIptr = derivIWinBuf.ptr<short>(y);

                    x = 0;
                    for (; x < winSize.width * cn; x++, dsrc += 2, dIptr += 2)
                    {
                        //Get sub pixel values from images
                        int ival = CV_DESCALE(src[x] * iw00 + src[x + cn] * iw01 +
                                                  src[x + stepI] * iw10 + src[x + stepI + cn] * iw11,
                                              W_BITS1 - 5);
                        int ixval = CV_DESCALE(dsrc[0] * iw00 + dsrc[cn2] * iw01 +
                                                   dsrc[dstep] * iw10 + dsrc[dstep + cn2] * iw11,
                                               W_BITS1);
                        int iyval = CV_DESCALE(dsrc[1] * iw00 + dsrc[cn2 + 1] * iw01 + dsrc[dstep + 1] * iw10 +
                                                   dsrc[dstep + cn2 + 1] * iw11,
                                               W_BITS1);

                        //Store values to the window buffers
                        Iptr[x] = (short)ival;
                        dIptr[0] = (short)ixval;
                        dIptr[1] = (short)iyval;

                        //Compute accum values for later gain and bias computation
                        meanI += (float)ival;
                        meanI2 += (float)(ival * ival);
                    }
                }
                //Compute means for later gain and bias computation
                vMeanI[level].push_back((meanI * FLT_SCALE) / winSize.area());
                vMeanI2[level].push_back((meanI2 * FLT_SCALE) / winSize.area());

                Iref[level].push_back(IWinBuf.clone());
                Idref[level].push_back(derivIWinBuf.clone());

                if (level == maxLevel)
                {
                    vpUpdatedMPs.push_back(vpNewMPs[i]);
                    vUpdatedKeys.push_back(vNewKeys[i]);
                    prevPts.push_back(vNewKeys[i]);
                    vpNewMPs[i]->trackedByKLT = true;
                }
            }
        }
    }
}