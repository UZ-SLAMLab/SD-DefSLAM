#include "DeformationViewer.h"
#include "DeformationFrameDrawer.h"
#include "DeformationMapDrawer.h"

#include "DeformationTracking.h"
#include "Tracking.h"
#include <iomanip>
#include <mutex>
#include <pangolin/pangolin.h>
#include <unistd.h>

namespace defSLAM {
Viewer::Viewer(System *pSystem, FrameDrawer *pFrameDrawer,
               MapDrawer *pMapDrawer, Tracking *pTracking,
               const string &strSettingPath)
    : ORB_SLAM2::Viewer(pSystem, pFrameDrawer, pMapDrawer, pTracking,
                        strSettingPath) {
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
  RegLap = fSettings["Regularizer.laplacian"];
  RegInex = fSettings["Regularizer.Inextensibility"];
  RegTemp = fSettings["Regularizer.temporal"];
}

void Viewer::Run() {
  mbFinished = false;
  mbStopped = false;

  pangolin::CreateWindowAndBind("DefSLAM: Map Viewer", 175 + 640, 480);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(175));
  pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
  pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
  pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", false, true);
  pangolin::Var<bool> menuShowGraph("menu.Show Graph", false, true);
  pangolin::Var<bool> menuShowError("menu.Show Error", false, true);
  pangolin::Var<bool> menuAutoPlay("menu.Autoplay", true, true);

  // Added for non-rigid method
  pangolin::Var<bool> menuShowTemplate("menu.Show Template", true, true);
  pangolin::Var<bool> menuShowTemplatehist("menu.Show History", false, true);

  pangolin::Var<bool> menuShowTexture("menu.Show Texture", true, true);
  pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false,
                                           true);
  pangolin::Var<double> menuInextensibility("menu.Inext", RegInex, 1, 100000,
                                            true);
  pangolin::Var<double> menuLaplacian("menu.Lapl", RegLap, 0.0001, 50000, true);
  pangolin::Var<double> menuTemporal("menu.Temp", RegTemp, 0, 1, false);

  pangolin::Var<bool> menuNext("menu.Next", false, false);
  pangolin::Var<bool> menuReset("menu.Reset", false, false);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(
      1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000);
  //  pangolin::OpenGlMatrix proj2 =
  //  pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000);

  pangolin::OpenGlRenderState s_cam(
      proj, pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ,
                                      (mViewpointX + (3)), mViewpointY + 0.1,
                                      mViewpointZ + 2, 0.0, -1, 0.0));

  // Add named OpenGL viewport to window and provide 3D Handler
  // Output 3D
  pangolin::View &d_cam1 =
      pangolin::CreateDisplay()
          .SetAspect(640.0f / 480.0f)
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1, -1024.0f / 768.0f)
          .SetHandler(new pangolin::Handler3D(s_cam));
  //

  // Add named OpenGL viewport to window and provide 3D Handler
  // Template Shape-at-rest
  /*pangolin::View& d_cam2 = pangolin::CreateDisplay()
          .SetAspect(640.0f/480.0f)
          .SetBounds(0.5, 1.0, .5, 1.0, -1024.0f/768.0f)
          .SetHandler(new pangolin::Handler3D(t_cam));*/

  pangolin::OpenGlMatrix Twc;
  Twc.SetIdentity();

  cv::namedWindow("DefSLAM: Current Frame");

  bool bFollow = true;
  bool bLocalizationMode = false;

  while (1) {
    usleep(mT * 1000);
    // sleep(1);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

    if (menuFollowCamera && bFollow) {
      s_cam.Follow(Twc);
    } else if (menuFollowCamera && !bFollow) {
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
          mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
      s_cam.Follow(Twc);
      bFollow = true;
    } else if (!menuFollowCamera && bFollow) {
      bFollow = false;
    }

    if (menuLocalizationMode && !bLocalizationMode) {
      mpSystem->ActivateLocalizationMode();
      bLocalizationMode = true;
    } else if (!menuLocalizationMode && bLocalizationMode) {
      mpSystem->DeactivateLocalizationMode();
      bLocalizationMode = false;
    }

    d_cam1.Activate(s_cam);

    glClearColor(0.5f, 0.5f, 0.5f, 0.f);
    mpMapDrawer->DrawCurrentCamera(Twc);
    if (menuShowKeyFrames || menuShowGraph)
      mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
    if (menuShowPoints) {
      mpMapDrawer->DrawPoints();
      if (menuShowError)
        mpMapDrawer->DrawGTPoints();
    }

    static_cast<DeformationMapDrawer *>(mpMapDrawer)
        ->ShowTexture(menuShowTexture);

    if ((menuShowTemplate)) {
      static_cast<DeformationMapDrawer *>(mpMapDrawer)->DrawTemplate(timestamp);
      if (menuShowTemplatehist)
        static_cast<DeformationMapDrawer *>(mpMapDrawer)->DrawTemplatehist();
    }
    if (mbSaveResults) {
      std::ostringstream out;
      out << std::internal << std::setfill('0') << std::setw(5)
          << uint(timestamp);
      d_cam1.SaveOnRender("3D" + out.str());
    }
    cv::Mat im = mpFrameDrawer->DrawFrame();
    if (!im.empty()) { //{&&!mpTracker->mCurrentFrame.ImOut.empty()){
      if (mbSaveResults) {
        std::ostringstream out;
        out << std::internal << std::setfill('0') << std::setw(5)
            << uint(timestamp);
        cv::imwrite("2D" + out.str() + ".png", im);
      }
      cv::imshow("DefSLAM: Current Frame", im);
      cv::waitKey(10);
    }

    // pangolin::FinishFrame();

    /*d_cam2.Activate(t_cam);
    if ((menuShowTemplate)) {
      static_cast<DeformationMapDrawer *>(mpMapDrawer)
          ->DrawTemplateAtRest(timestamp);
    }

    if (menuShowPoints) {
      static_cast<DeformationMapDrawer *>(mpMapDrawer)->DrawPointsAtRest();
    }*/

    pangolin::FinishFrame();
    unique_lock<mutex> lock22(mMutexTimeStamp);
    mpTracker->setRegInex(menuInextensibility);
    mpTracker->setRegLap(menuLaplacian);
    mpTracker->setRegTemp(menuTemporal);

    if ((menuNext) or (menuAutoPlay)) {
      unique_lock<mutex> locknext(mMutexNext);
      this->next = true;
      menuNext = false;
    } else {
      unique_lock<mutex> locknext(mMutexNext);
      this->next = false;
    }

    if (menuReset) {
      menuShowGraph = true;
      menuShowKeyFrames = false;
      menuShowPoints = true;
      menuLocalizationMode = false;
      if (bLocalizationMode)
        mpSystem->DeactivateLocalizationMode();
      bLocalizationMode = false;
      bFollow = true;
      menuFollowCamera = false;
      mpSystem->Reset();
      menuReset = false;
    }

    if (Stop()) {
      while (isStopped()) {
        usleep(3000);
      }
    }

    if (CheckFinish())
      break;
  }
  // MeshDrawer::Exit();
  SetFinish();
}
}
