/*
 * This file is part of SD-DefSLAM
 * Copyright (C) 2020 Juan J. Gómez Rodríguez, Jose Lamarca Peiro, J. Morlana,
 *                    Juan D. Tardós and J.M.M. Montiel, University of Zaragoza
 *
 * This software is for internal use in the EndoMapper project.
 * Not to be re-distributed.
 */


#ifndef SETTINGLOADER_H
#define SETTINGLOADER_H

#include <string>
#include <opencv2/core/core.hpp>

namespace defSLAM
{
    class SettingsLoader
    {
    public:
        // Constructor by default. You have to load the files manually.
        SettingsLoader();
        // Constructor from file. You load the files from a file.
        SettingsLoader(const std::string &strSettingsFile);

    public:
        int getFPS() const;

        int getCameraWidth() const;
        int getCameraHeight() const;
        float getViewPointX() const;
        float getViewPointY() const;
        float getViewPointZ() const;
        float getViewPointF() const;
        bool getSaveResults() const;

        float getkeyFrameSize() const;
        float getkeyFrameLineWidth() const;
        float getgraphLineWidth() const;
        float getpointSize() const;
        float getcameraSize() const;
        float getcameraLineWidth() const;

        cv::Mat getK() const; // Intrinsic Calibration Matrix
        cv::Mat getdistCoef() const;
        float getbf() const;
        int getminFrames() const;
        int getmaxFrames() const;
        int getRGB() const;
        int getnFeatures() const;
        float getfScaleFactor() const;
        int getnLevels() const;
        int getfIniThFAST() const;
        int getfMinThFAST() const;
        float getregLap() const;               // Laplacian
        float getregTemp() const;              // Temporal
        float getregStreching() const;         // Streching
        float getLocalZone() const;            // threshold of reliability
        float getreliabilityThreshold() const; // threshold of reliability
        int getpointsToTemplate() const;
        float getchiLimit() const;
        float getschwarpReg() const;
        float getbendingReg() const;
        float getfps() const;
        float getT() const;
        std::string getFilterPath() const;
        bool getDebugPoints() const;

    public:
        void setSaveResults(const bool);
        void setK(const cv::Mat &k);
        void setD(const cv::Mat &D);
        void setbf(const float);
        void setCameraWidth(const int);
        void setCameraHeight(const int);
        void setFilterPath(const std::string &s);

    private:
        // Regularizers
        float regLap_;               // Laplacian
        float regTemp_;              // Temporal
        float regInex_;              // Streching
        double localZoneSize_;       // local zone size
        float reliabilityThreshold_; // threshold of reliability
        bool saveResults_;           // Saving the results
        bool debugPoints_;           // Debug current points
        // Tracking
        cv::Mat K_;        // Intrinsic Calibration Matrix
        cv::Mat distCoef_; //distorsion coefficients
        float bf_;
        int minFrames_;
        int maxFrames_;
        int RGB_;
        int nFeatures_;
        float fScaleFactor_;
        int nLevels_;
        int fIniThFAST_;
        int fMinThFAST_;
        // Local Mapping
        int pointsToTemplate_;
        float chiLimit_;
        float schwarpReg_;
        float bendingReg_;
        float fps_;
        float T_;
        // Viewer
        int imageWidth_;
        int imageHeight_;
        float viewpointX_;
        float viewpointY_;
        float viewpointZ_;
        float viewpointF_;
        // Map Drawer
        float keyFrameSize_;
        float keyFrameLineWidth_;
        float graphLineWidth_;
        float pointSize_;
        float cameraSize_;
        float cameraLineWidth_;
        //Filter model path
        std::string filterPath_;
    };
} // namespace defSLAM
#endif