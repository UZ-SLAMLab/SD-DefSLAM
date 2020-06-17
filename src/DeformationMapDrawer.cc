#include "DeformationMapDrawer.h"
#include "DeformationKeyFrame.h"
#include "DeformationMap.h"
#include "DeformationMapPoint.h"
#include "Edge.h"
#include "Facet.h"
#include "GroundTruthFrame.h"
#include "GroundTruthKeyFrame.h"
#include "KeyFrame.h"
#include "MeshDrawer.h"
#include "Node.h"
#include "Template.h"
#include <mutex>
#include <pangolin/pangolin.h>

namespace defSLAM
{

  DeformationMapDrawer::DeformationMapDrawer(Map *pMap,
                                             const string &strSettingPath)
      : MapDrawer(pMap, strSettingPath), MeshDrawers(nullptr)
  {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mTemplateLineWidth = fSettings["Viewer.TemplateLineWidth"];
  }

  void DeformationMapDrawer::reset() { this->MeshDrawershist.clear(); }

  void DeformationMapDrawer::UpdateTemplate()
  {
    std::unique_lock<std::mutex> M(this->mTemplate);
    if (static_cast<DeformationMap *>(mpMap)->ThereIsATemplate)
    {
      std::unique_lock<std::mutex> lc(
          static_cast<DeformationMap *>(mpMap)->GetTemplate()->MutexUpdating);

      if (this->MeshDrawers)
      {
        delete this->MeshDrawers;
      }

      this->MeshDrawers = new MeshDrawer();
      cv::Mat imRGB =
          static_cast<DeformationMap *>(mpMap)->GetTemplate()->getTexture();

      if ((Texture) && (!imRGB.empty()))
      {
        this->MeshDrawers->addTextureImage(imRGB);
      }
      else
      {
        cv::Mat imRGBNoTex(1024, 1024, CV_8UC3, cv::Scalar(155, 155, 155));
        this->MeshDrawers->addTextureImage(imRGBNoTex);
      }

      std::set<Node *> Nodes =
          static_cast<DeformationMap *>(mpMap)->GetTemplate()->get_Nodes();
      std::vector<Node *> NodesVec(Nodes.begin(), Nodes.end());

      for (uint i(0); i < NodesVec.size(); i++)
      {
        std::vector<double> pos;
        std::vector<double> proj;
        pos.push_back(NodesVec[i]->x);
        pos.push_back(NodesVec[i]->y);
        pos.push_back(NodesVec[i]->z);
        proj.push_back(NodesVec[i]->proju / imRGB.cols);
        proj.push_back(NodesVec[i]->projv / imRGB.rows);
        NodesVec[i]->PosVecDrawer = i;

        if (static_cast<Node *>(NodesVec[i])->isBoundary())
        {
          this->MeshDrawers->AddNode(pos, proj, 0);
        }
        else
        {
          if (static_cast<Node *>(NodesVec[i])->isViewed())
          {
            this->MeshDrawers->AddNode(pos, proj, 1);
          }
          else if (static_cast<Node *>(NodesVec[i])->isLocal())
          {
            this->MeshDrawers->AddNode(pos, proj, 2);
          }
          else
          {
            this->MeshDrawers->AddNode(pos, proj, 4);
          }
        }
      }
      std::set<Facet *> Facets =
          static_cast<DeformationMap *>(mpMap)->GetTemplate()->get_Facets();

      for (std::set<Facet *>::iterator itf = Facets.begin(); itf != Facets.end();
           itf++)
      {
        std::vector<int> fac;
        std::set<Node *> Nodes = (*itf)->getNodes();
        for (std::set<Node *>::iterator it = Nodes.begin(); it != Nodes.end();
             it++)
        {
          fac.push_back((*it)->PosVecDrawer);
        }

        this->MeshDrawers->AddFacet(fac);
      }

      // Draw Edges
      std::set<Edge *> Edges =
          static_cast<DeformationMap *>(mpMap)->GetTemplate()->get_Edges();
      for (std::set<Edge *>::iterator ite = Edges.begin(); ite != Edges.end();
           ite++)
      {
        std::set<Node *> Nodesedge = (*ite)->get_nodes();
        std::vector<int> edge;
        for (std::set<Node *>::iterator it = Nodesedge.begin();
             it != Nodesedge.end(); it++)
        {
          edge.push_back((*it)->PosVecDrawer);
        }
        this->MeshDrawers->AddEdge(edge);
      }
    }
  }

  void DeformationMapDrawer::UpdateTemplateAtRest()
  {
    std::unique_lock<std::mutex> M(this->mTemplatehist);
    if (static_cast<DeformationMap *>(mpMap)->ThereIsATemplate)
    {
      auto defMap = static_cast<DeformationMap *>(mpMap);
      auto templateToDraw = defMap->GetTemplate();
      auto refKf = templateToDraw->kf;

      std::unique_lock<std::mutex> lc(templateToDraw->MutexUpdating);
      this->MeshDrawershist[refKf] = unique_ptr<MeshDrawer>(new MeshDrawer());
      MeshDrawer *temp = this->MeshDrawershist[refKf].get();
      cv::Mat imRGB = templateToDraw->getTexture();

      if ((Texture) && (!imRGB.empty()))
      {
        temp->addTextureImage(imRGB);
      }
      else
      {
        cv::Mat imRGBNoTex(1024, 1024, CV_8UC3, cv::Scalar(155, 155, 155));
        temp->addTextureImage(imRGBNoTex);
      }

      std::set<Node *> Nodes = templateToDraw->get_Nodes();
      std::vector<Node *> NodesVec(Nodes.begin(), Nodes.end());

      for (uint i(0); i < NodesVec.size(); i++)
      {
        std::vector<double> pos;
        std::vector<double> proj;
        pos.push_back(NodesVec[i]->xO);
        pos.push_back(NodesVec[i]->yO);
        pos.push_back(NodesVec[i]->zO);
        proj.push_back(NodesVec[i]->proju / imRGB.cols);
        proj.push_back(NodesVec[i]->projv / imRGB.rows);
        NodesVec[i]->PosVecDrawer = i;
        temp->AddNode(pos, proj, 4);
      }
      std::set<Facet *> Facets = templateToDraw->get_Facets();

      for (std::set<Facet *>::iterator itf = Facets.begin(); itf != Facets.end();
           itf++)
      {
        std::vector<int> fac;
        std::set<Node *> Nodes = (*itf)->getNodes();
        for (std::set<Node *>::iterator it = Nodes.begin(); it != Nodes.end();
             it++)
        {
          fac.push_back((*it)->PosVecDrawer);
        }
        temp->AddFacet(fac);
      }

      // Draw Edges
      std::set<Edge *> Edges = defMap->GetTemplate()->get_Edges();
      for (std::set<Edge *>::iterator ite = Edges.begin(); ite != Edges.end();
           ite++)
      {
        std::set<Node *> Nodesedge = (*ite)->get_nodes();
        std::vector<int> edge;
        for (std::set<Node *>::iterator it = Nodesedge.begin();
             it != Nodesedge.end(); it++)
        {
          edge.push_back((*it)->PosVecDrawer);
        }
        temp->AddEdge(edge);
      }
      PointsRef.clear();
      for (uint i(0); i < refKf->mvKeys.size(); i++)
      {
        MapPoint *pMP = refKf->GetMapPoint(i);
        if (pMP)
        {
          if (pMP->isBad())
            continue;

          if (static_cast<DeformationMapPoint *>(pMP)->getFacet())
          {
            if (!pMP->covNorm)
              continue;
            cv::Mat x3Dy = static_cast<DeformationMapPoint *>(pMP)
                               ->PosesKeyframes[refKf]
                               .clone();
            if (x3Dy.empty())
              continue;
            PointsRef.push_back(x3Dy.at<float>(0, 0));
            PointsRef.push_back(x3Dy.at<float>(1, 0));
            PointsRef.push_back(x3Dy.at<float>(2, 0));
          }
        }
      }
    }
  }

  void DeformationMapDrawer::UpdatePointsAtRest(KeyFrame *pFrame, float s)
  {
    std::unique_lock<std::mutex> K(mPointsar);
    // unique_lock<mutex> lock(mMutex);

    PointsSeenar.clear();
    PointsMonoar.clear();
    PointsGTar.clear();
    // int stereoPoints=0;*/
    // std::vector<MapPoint*> AllMapPoints = mpMap->GetAllMapPoints();

    // std::vector<MapPoint*> MapPoints = pFrame->mvpMapPoints;
    std::vector<cv::Point3f> LocalPoints =
        static_cast<GroundTruthKeyFrame *>(pFrame)->mvLocalMapPoints;
    std::vector<cv::Point3f> StereoPoints =
        static_cast<GroundTruthKeyFrame *>(pFrame)->mvStereoMapPoints;

    cv::Mat Twc = pFrame->GetPoseInverse();
    cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3).clone();
    cv::Mat twc = Twc.rowRange(0, 3).col(3).clone();

    for (size_t i = 0, iend = LocalPoints.size(); i < iend; i++)
    {
      cv::Mat mono_c = (cv::Mat_<float>(3, 1) << LocalPoints[i].x,
                        LocalPoints[i].y, LocalPoints[i].z);
      cv::Mat stereo_c = (cv::Mat_<float>(3, 1) << StereoPoints[i].x,
                          StereoPoints[i].y, StereoPoints[i].z);

      cv::Mat mono_c_scaled = mono_c * s;

      cv::Mat mono_w = Rwc * mono_c + twc;
      cv::Mat mono_w_scaled = Rwc * mono_c_scaled + twc;
      cv::Mat stereo_w = Rwc * stereo_c + twc;

      PointsMonoar.push_back(mono_w.at<float>(0));
      PointsMonoar.push_back(mono_w.at<float>(1));
      PointsMonoar.push_back(mono_w.at<float>(2));

      PointsSeenar.push_back(mono_w_scaled.at<float>(0));
      PointsSeenar.push_back(mono_w_scaled.at<float>(1));
      PointsSeenar.push_back(mono_w_scaled.at<float>(2));

      PointsGTar.push_back(stereo_w.at<float>(0));
      PointsGTar.push_back(stereo_w.at<float>(1));
      PointsGTar.push_back(stereo_w.at<float>(2));
    }
  }

  void DeformationMapDrawer::DrawPointsAtRest()
  {
    std::unique_lock<std::mutex> K(mPointsar);

    glPointSize(5);
    glBegin(GL_POINTS);
    glColor3f(0.0, 1.0, 0.0); // RGB

    for (size_t i = 0, iend = PointsSeenar.size(); i < iend; i = i + 3)
    {
      glVertex3f(PointsSeenar[i], PointsSeenar[i + 1],
                 PointsSeenar[i + 2]); // Green points: mono points scaled
    }
    glEnd();

    glPointSize(5);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 1.0); // RGB

    for (size_t i = 0, iend = PointsMonoar.size(); i < iend; i = i + 3)
    {
      glVertex3f(PointsMonoar[i], PointsMonoar[i + 1],
                 PointsMonoar[i + 2]); // Blue points: mono points
    }
    glEnd();

    /*  glPointSize(5);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,1.0); //RGB

    for(size_t i=0, iend=PointsStereoFrame.size(); i<iend;i=i+3)
    {
        glVertex3f(PointsStereoFrame[i],PointsStereoFrame[i+1],PointsStereoFrame[i+2]);
    //Pink points: mono points
    }
    glEnd();
*/
    /* glPointSize(mPointSize);
   glBegin(GL_POINTS);
   glColor3f(1.0,0.0,0.0); //RGB

   for(size_t i=0, iend=PointsLocal.size(); i<iend;i=i+3)
   {
       glVertex3f(PointsLocalar[i],PointsLocalar[i+1],PointsLocalar[i+2]); //Red
   points: points lying in facet
   }
   glEnd();
*/
    glPointSize(5);
    glBegin(GL_POINTS);
    glColor3f(1.0, 1.0, 0.0);

    for (size_t i = 0, iend = PointsGTar.size(); i < iend; i = i + 3)
    {
      glVertex3f(PointsGTar[i], PointsGTar[i + 1],
                 PointsGTar[i + 2]); // Yellow points: stereo points (GT)
    }
    glEnd();

    // Draw error lines
    float lambda = 1.0;
    for (size_t i = 0, iend = PointsSeenar.size(); i < iend; i = i + 3)
    {
      glLineWidth(mKeyFrameLineWidth); //++3
      glColor3f(0.0, 0.0, 1.0);
      glBegin(GL_LINES);

      // Sometimes stereo is an empty cv::Mat-->need to be solved
      if (PointsGTar[i + 2] > 0.0)
      {

        glVertex3f(PointsSeenar[i], PointsSeenar[i + 1], PointsSeenar[i + 2]);
        float v1, v2, v3;
        v1 = PointsSeenar[i] + lambda * (PointsGTar[i] - PointsSeenar[i]);
        v2 = PointsSeenar[i + 1] +
             lambda * (PointsGTar[i + 1] - PointsSeenar[i + 1]);
        v3 = PointsSeenar[i + 2] +
             lambda * (PointsGTar[i + 2] - PointsSeenar[i + 2]);
        glVertex3f(v1, v2, v3);
      }

      glEnd();
    }
  }

  void DeformationMapDrawer::DrawTemplate(uint o)
  {
    std::unique_lock<std::mutex> M(this->mTemplate);
    if (this->MeshDrawers)
    {
      this->MeshDrawers->DrawMesh();
    }
  }

  void DeformationMapDrawer::DrawTemplatehist()
  {
    std::unique_lock<std::mutex> M(this->mTemplatehist);
    for (auto &it : this->MeshDrawershist)
    {
      double alpha(0.99);
      it.second->DrawMesh(alpha, false);
    }
  }

  void DeformationMapDrawer::DrawTemplateAtRest(uint o)
  {
    std::unique_lock<std::mutex> M(
        static_cast<DeformationMap *>(mpMap)->MutexUpdating);

    if (static_cast<DeformationMap *>(mpMap)->ThereIsATemplate)
    {
      std::unique_lock<std::mutex> lc(
          static_cast<DeformationMap *>(mpMap)->GetTemplate()->MutexUpdating);

      std::set<Facet *> Facets =
          static_cast<DeformationMap *>(mpMap)->GetTemplate()->get_Facets();
      uint j(0);

      if (Texture)
      {
        glEnable(GL_TEXTURE_2D);

        for (std::map<KeyFrame *, std::set<Facet *>>::iterator itKf =
                 Facet::TexturesDataset.begin();
             itKf != Facet::TexturesDataset.end(); itKf++)
        {
          GLuint texID;
          glGenTextures(1, &texID);
          glBindTexture(GL_TEXTURE_2D, texID);
          glPixelStoref(GL_UNPACK_ALIGNMENT, 1);
          glLineWidth(1);

          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

          glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, itKf->first->RGBimage.cols,
                       itKf->first->RGBimage.rows, 0, GL_BGR, GL_UNSIGNED_BYTE,
                       itKf->first->RGBimage.ptr());

          // std::cout << "size 2" << itKf->second.size() << std::endl;
          for (std::set<Facet *>::iterator itf = itKf->second.begin();
               itf != itKf->second.end(); itf++)
          {
            glBegin(GL_TRIANGLES);
            glColor3f(1, 1, 1); // set global color to white, otherwise this color
                                // will be (somehow) added to the texture

            // glColor3d((*itf)->R,(*itf)->G,(*itf)->B);
            std::set<Node *> Nodes = (*itf)->getNodes();
            for (std::set<Node *>::iterator it = Nodes.begin(); it != Nodes.end();
                 it++)
            {
              cv::Point UV = (*itf)->texture.second[(*it)];
              glTexCoord2f(((float)UV.x) / itKf->first->RGBimage.cols,
                           ((float)UV.y) / itKf->first->RGBimage.rows);
              glVertex3f((*it)->xO, (*it)->yO, (*it)->zO);
            }
          }
          glEnd();
          glDeleteTextures(1, &texID); // TODO load all textures once and only
                                       // remove when it change
        }

        glDisable(GL_TEXTURE_2D);

        for (std::set<Facet *>::iterator itf = Facets.begin();
             itf != Facets.end(); itf++)
        {
          // Facets
          if (!(*itf)->texture.first)
          {
            /*  glBegin(GL_TRIANGLES);
            glColor3d((*itf)->R,(*itf)->G,(*itf)->B);
            std::array<Node*, 3>  Nodes((*itf)->getNodesArray());

            for(uint i(0);i<3;i++){
                glVertex3f((Nodes[i])->x,(Nodes[i])->y,(Nodes[i])->z);
                j++;
            }*/
          }
        }
      }

      // Draw Edges
      std::set<Edge *> Edges =
          static_cast<DeformationMap *>(mpMap)->GetTemplate()->get_Edges();
      for (std::set<Edge *>::iterator ite = Edges.begin(); ite != Edges.end();
           ite++)
      {
        std::set<Node *> Nodesedge = (*ite)->get_nodes();
        glBegin(GL_LINES);

        for (std::set<Node *>::iterator it = Nodesedge.begin();
             it != Nodesedge.end(); it++)
        {
          if (static_cast<Node *>(*it)->isBoundary())
          {
            glColor3f(0.1, 0, 1);
            glVertex3f((*it)->xO, (*it)->yO, (*it)->zO);
          }
          else
          {
            if (static_cast<Node *>(*it)->isViewed())
            {
              glColor3f(0.1, 0.9, 0.1);
              glVertex3f((*it)->xO, (*it)->yO, (*it)->zO);
            }
            else if (static_cast<Node *>(*it)->isLocal())
            {
              glColor3f(1.0, 0.8, 0.1);
              glVertex3f((*it)->xO, (*it)->yO, (*it)->zO);
            }
            else
            {
              glColor3f(0.05, 0.05, 0.05);
              glVertex3f((*it)->xO, (*it)->yO, (*it)->zO);
            }
          }
        }
        glEnd();
      }
    }
  }

  void DeformationMapDrawer::ShowCurvature(bool C)
  {
    std::unique_lock<std::mutex> mL(mcurvature);
    Curvature = C;
  }
  void DeformationMapDrawer::ShowTexture(bool C)
  {
    std::unique_lock<std::mutex> mL(mTexture);
    Texture = C;
  }

  void DeformationMapDrawer::DrawKeyFrames(const bool bDrawKF,
                                           const bool bDrawGraph)
  {
    const float &w = mKeyFrameSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    const vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();

    if (bDrawKF)
    {
      for (size_t i = 0; i < vpKFs.size(); i++)
      {
        KeyFrame *pKF = vpKFs[i];
        cv::Mat Twc = pKF->GetPoseInverse().t();

        glPushMatrix();

        glMultMatrixf(Twc.ptr<GLfloat>(0));

        glLineWidth(mKeyFrameLineWidth);
        if (static_cast<DeformationKeyFrame *>(pKF)->templateAssigned())
          glColor3f(0.0f, 0.0f, 1.0f);
        else
          glColor3f(1.0f, 0.0f, 0.0f);

        if ((pKF) == static_cast<DeformationMap *>(mpMap)->getLastKFTemplate())
          glColor3f(0.0f, 1.0f, 0.0f);

        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
      }
    }

    if (bDrawGraph)
    {
      glLineWidth(mGraphLineWidth);
      glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
      glBegin(GL_LINES);

      for (size_t i = 0; i < vpKFs.size(); i++)
      {
        // Covisibility Graph
        const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
        cv::Mat Ow = vpKFs[i]->GetCameraCenter();
        if (!vCovKFs.empty())
        {
          for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(),
                                                  vend = vCovKFs.end();
               vit != vend; vit++)
          {
            if ((*vit)->mnId < vpKFs[i]->mnId)
              continue;
            cv::Mat Ow2 = (*vit)->GetCameraCenter();
            glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
            glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
          }
        }

        // Spanning tree
        KeyFrame *pParent = vpKFs[i]->GetParent();
        if (pParent)
        {
          cv::Mat Owp = pParent->GetCameraCenter();
          glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
          glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
        }

        // Loops
        set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
        for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(),
                                       send = sLoopKFs.end();
             sit != send; sit++)
        {
          if ((*sit)->mnId < vpKFs[i]->mnId)
            continue;
          cv::Mat Owl = (*sit)->GetCameraCenter();
          glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
          glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
        }
      }

      glEnd();
    }
  }
} // namespace defSLAM