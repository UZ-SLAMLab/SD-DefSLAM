
#include "Examples/dataset/SystemTest.h"
#include "Converter.h"
#include "DeformationTracking.h"
#include "DeformationMap.h"
#include "DeformationMapDrawer.h"
#include "DeformationFrameDrawer.h"

#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

namespace ORB_SLAM2
{

SystemTest::SystemTest(const string &strVocFile, const string &strSettingsFile, std::vector<std::vector<double>> & vertex, std::vector<std::vector<int> > &indexes,
               const bool bUseViewer)
     /*mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(true),
        mbDeactivateLocalizationMode(false)*/
{
    // Output welcome message
    cout << endl <<
    "ORB-SDTAM 2017-2018 José Lamarca, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl << endl;

    cout << "Input sensor was set to: ";

        cout << "Monocular" << endl;
    mpViewer = static_cast<Viewer*>(NULL);
    mbActivateLocalizationMode =true;
    mbReset=(false);
    mbDeactivateLocalizationMode=(false);
    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new DeformationMap(vertex,indexes);

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new DeformationFrameDrawer(mpMap);
    mpMapDrawer = new DeformationMapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new DeformationTracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }


    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, true);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, false);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);


    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}
void SystemTest::setMatches3D(std::vector<std::vector<double>> & matches){
    Matches3D =matches;
}

cv::Mat SystemTest::TrackMonocular(const cv::Mat &im, std::vector<std::vector<double>> & matches, const double &timestamp,const cv::Mat & TcwO)
{
    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    outliers.reserve(matches.size());
   static_cast<DeformationTracking*>(mpTracker)->TrackDataset(im,matches,timestamp,TcwO);


    /*for (uint i(0);i<outliers.size();i++){
        outliers[i]=outliers_aux[i];
    }*/
    if (mpViewer)
        mpViewer->Updatetimestamp(timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    cv::Mat Tcw;
    return Tcw;
}


} //namespace ORB_SLAM
