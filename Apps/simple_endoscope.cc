/*
 * This file is part of SD-DefSLAM
 * Copyright (C) 2020 Juan J. Gómez Rodríguez, Jose Lamarca Peiro, J. Morlana,
 *                    Juan D. Tardós and J.M.M. Montiel, University of Zaragoza
 *
 * This software is for internal use in the EndoMapper project.
 * Not to be re-distributed.
 */

#include <opencv2/core/core.hpp>
#include <System.h>
#include <SettingsLoader.h>
#include <opencv2/calib3d.hpp>

int main(int argc, char **argv)
{
    std::cout << argc << std::endl;
    if ((argc != 4) and (argc != 3))
    {
        cerr << endl
             << "Usage: ./DefSLAM ORBvocabulary calibrationFile video" << endl
             << " or    ./DefSLAM ORBvocabulary calibrationFile  " << endl
             << endl;
        return 1;
    }

    cv::VideoCapture cap;
    if (argc == 3)
        cap.open(0);
    else
        cap.open(argv[3]);

    if (!cap.isOpened()) // check if we succeeded
        return -1;

    string settingFile = argv[2];
    string vocabularyFile = argv[1];
    defSLAM::SettingsLoader settingsLoader(settingFile);

    cv::FileStorage fsSettings(settingFile, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << settingFile << endl;
        exit(-1);
    }

    cv::Vec4f D;

    float k1 = fsSettings["Camera.k1"];
    float k2 = fsSettings["Camera.k2"];
    float k3 = fsSettings["Camera.k3"];
    float k4 = fsSettings["Camera.k4"];
    D(0) = k1;
    D(1) = k2;
    D(2) = k3;
    D(3) = k4;

    cv::Mat K = settingsLoader.getK().clone();
    std::cout << "Distorsion Vector : " << D << std::endl;
    std::cout << "Calibration Matrix : " << K << std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames.
    cv::Mat I = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat map1, map2;
    cv::Size dim(settingsLoader.getCameraWidth(), settingsLoader.getCameraHeight());

    cv::Mat K_est;
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, dim, I, K_est, 1);
    //Estima la K despues de rectificar la imagen
    std::cout
        << "K_est: " << K_est << std::endl;
    // Focals keep the same because the angle covered for each pixel is equal.
    // You have to substract the half of the crop to the optical center

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = 0;
    DistCoef.at<float>(1) = 0;
    DistCoef.at<float>(2) = 0;
    DistCoef.at<float>(3) = 0;
    settingsLoader.setD(DistCoef);
    cv::fisheye::initUndistortRectifyMap(K, D, I, K_est, dim, CV_32FC1, map1, map2);
    //Desdistorsiona las coordenadas y las mete en los map
    uint i(-1);
    int cropy(700);
    int cropx(800);
    cv::Rect myROI(cropx / 2, cropy / 2, settingsLoader.getCameraWidth() - cropx, settingsLoader.getCameraHeight() - cropy);

    K_est.at<float>(0, 2) = K_est.at<float>(0, 2) - cropx / 2;
    K_est.at<float>(1, 2) = K_est.at<float>(1, 2) - cropy / 2;
    settingsLoader.setK(K_est);
    //cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    defSLAM::System SLAM(vocabularyFile, settingsLoader, true);

    uint initial_point(14 * 40 * 60);
    cap.set(cv::CAP_PROP_POS_FRAMES, initial_point);
    while (cap.isOpened())
    {
        cv::Mat imDistort;
        cap >> imDistort;
        i++;
        /*if (i < initial_point)
        {
            std::cout << "Loading : " << (double(i) / initial_point) * 100 << "% " << '\r' << std::flush;
            continue;
        }*/
        if (imDistort.empty())
        {
            cerr << endl
                 << "Failed to load image at: " << to_string(i) << endl;
            return 1;
        }
        //cv::imshow("original image", imDistort);
        //cv::waitKey(10);
        cv::Mat imUndistort;
        cv::remap(imDistort, imUndistort, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::Mat croppedImage = imUndistort(myROI).clone();
        //cv::imshow("original image", imUndistort);
        //cv::imshow("undistort image", croppedImage);
        //cv::waitKey(10);
        SLAM.TrackMonocular(croppedImage, i);
    }

    SLAM.Shutdown();

    return 0;
}
