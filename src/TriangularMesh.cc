#include "DeformationKeyFrame.h"
#include <TriangularMesh.h>
#include <algorithm>
#include <iterator>

namespace defSLAM
{
  using ORB_SLAM2::MapPoint;
  std::set<Node *> CalculateNodesToOptimize(const std::set<Node *> ViewedNodes,
                                            uint NN);
  TriangularMesh::TriangularMesh(std::set<MapPoint *> &mspMapPoints, Map *map)
      : Template(mspMapPoints, map, nullptr)
  {
    std::vector<std::vector<float>> vertex;
    std::vector<std::vector<int>> facets;
    this->SetNodes(vertex, facets);
    this->CalculateFeaturesCoordinates();
    this->deletepointsout();
    this->GetFacetTexture();
  }

  TriangularMesh::TriangularMesh(std::vector<std::vector<double>> &vertexes,
                                 std::vector<std::vector<int>> &indexes, Map *map)
      : Template(map)
  {
    std::vector<Node *> NodesIndex;
    for (uint j = 0; j < vertexes.size() - 1; j++)
    {
      Node *NewNode =
          new Node(vertexes[j][0], vertexes[j][1], vertexes[j][2], j, this);
      mspNodeTemplate.insert(NewNode);
      NodesIndex.push_back(NewNode);
      NodeDataSetArray.push_back(NewNode);
      NewNode = nullptr;
      numVertices++;
    }

    for (uint j = 0; j < indexes.size(); j++)
    {
      mspFacets.insert(new Facet(NodesIndex[size_t(indexes[j][0])],
                                 NodesIndex[size_t(indexes[j][1])],
                                 NodesIndex[size_t(indexes[j][2])], this));
    }
  }

  TriangularMesh::TriangularMesh(std::set<MapPoint *> &mspMapPoints, Map *map,
                                 KeyFrame *kf)
      : Template(mspMapPoints, map, kf)
  {
    Surface *RefSurface = static_cast<DeformationKeyFrame *>(kf)->surface;
    std::vector<cv::Mat> NodesSurface;
    RefSurface->getVertex(NodesSurface, 10, 10);

    std::vector<std::vector<float>> vertexW;
    std::vector<std::vector<float>> vertexC;

    cv::Mat Twc = kf->GetPoseInverse();
    vertexW.reserve(NodesSurface.size());
    vertexC.reserve(NodesSurface.size());

    for (uint i(0); i < NodesSurface.size(); i++)
    {
      std::vector<float> ptC, ptW;
      ptC.reserve(3);
      ptW.reserve(3);
      cv::Mat x3wh(4, 1, CV_32F);
      x3wh = Twc * NodesSurface[i];
      ptW.push_back(x3wh.at<float>(0, 0));
      ptW.push_back(x3wh.at<float>(1, 0));
      ptW.push_back(x3wh.at<float>(2, 0));
      vertexW.push_back(ptW);
      ptC.push_back(NodesSurface[i].at<float>(0, 0));
      ptC.push_back(NodesSurface[i].at<float>(1, 0));
      ptC.push_back(NodesSurface[i].at<float>(2, 0));
      vertexC.push_back(NodesSurface[i]);
    }
    Timer timer;
    timer.start();
    vtkSmartPointer<vtkPolyData> Mesh = this->DelaunayTriangulation(vertexC);
    timer.stop();
    /*std::cout << "trian : " << timer.getElapsedTimeInMilliSec() << "ms "
            << std::endl;*/
    timer.start();
    this->SetNodes(Mesh, vertexW);
    timer.stop();
    /*std::cout << "SetNodes : " << timer.getElapsedTimeInMilliSec() << "ms "
            << std::endl;*/
    timer.start();
    this->CalculateFeaturesCoordinates();
    timer.stop();
    /*  std::cout << "features : " << timer.getElapsedTimeInMilliSec() << "ms "
              << std::endl;*/
    timer.start();
    this->texture = kf->RGBimage.clone();
    this->GetFacetTexture(kf);
  }

  TriangularMesh::~TriangularMesh() { keyframeToFacet.clear(); }

  vtkSmartPointer<vtkPolyData> TriangularMesh::DelaunayTriangulation(
      std::vector<std::vector<float>> &vertexC)
  {
    using namespace std;
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    // Add the grid points to a polydata object
    for (uint i(0); i < vertexC.size(); i++)
    {
      points->InsertNextPoint(vertexC[i][0], vertexC[i][1], 1);
    }
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);
    // Triangulate the grid points
    vtkSmartPointer<vtkDelaunay2D> delaunay =
        vtkSmartPointer<vtkDelaunay2D>::New();
    delaunay->SetProjectionPlaneMode(0);
    delaunay->SetInputData(polydata);
    delaunay->Update();

    vtkSmartPointer<vtkPolyData> Mesh = delaunay->GetOutput();
    return Mesh;
  }

  void TriangularMesh::SetNodes(std::vector<std::vector<float>> &vertex,
                                std::vector<std::vector<int>> &facets)
  {
    std::vector<Node *> NodesIndex;
    NodesIndex.reserve(vertex.size());
    for (uint j = 0; j < vertex.size(); j++)
    {
      Node *NewNode = new Node(vertex[j][0], vertex[j][1], vertex[j][2], j, this);
      mspNodeTemplate.insert(NewNode);
      NodesIndex.push_back(NewNode);
      NewNode = nullptr;
      numVertices++;
    }
    facets.reserve(facets.size());
    for (uint j = 0; j < facets.size(); j++)
    {
      Facet *facet = new Facet(NodesIndex[size_t(facets[j][0])],
                               NodesIndex[size_t(facets[j][1])],
                               NodesIndex[size_t(facets[j][2])], this);
      mspFacets.insert(facet);
    }
  }

  void TriangularMesh::SetNodes(vtkSmartPointer<vtkPolyData> mesh,
                                std::vector<std::vector<float>> &vertexW)
  {
    numVertices = 0;
    std::vector<Node *> NodesIndex;
    vtkSmartPointer<vtkCellArray> Vertex = mesh->GetVerts();

    for (uint j = 0; j < vertexW.size(); j++)
    {
      Node *NewNode =
          new Node(vertexW[j][0], vertexW[j][1], vertexW[j][2], j, this);
      mspNodeTemplate.insert(NewNode);
      NodesIndex.push_back(NewNode);
      NewNode = nullptr;
      numVertices++;
    }

    vtkSmartPointer<vtkCellArray> FacesArray =
        vtkSmartPointer<vtkCellArray>::New();
    FacesArray = mesh->GetPolys();
    FacesArray->InitTraversal();
    vtkSmartPointer<vtkIdList> idList = vtkSmartPointer<vtkIdList>::New();
    while (FacesArray->GetNextCell(idList))
    {
      int a = idList->GetId(0);
      int b = idList->GetId(1);
      int c = idList->GetId(2);
      mspFacets.insert(new Facet(static_cast<Node *>(NodesIndex[a]),
                                 static_cast<Node *>(NodesIndex[b]),
                                 static_cast<Node *>(NodesIndex[c]), this));
    }
  }

  void TriangularMesh::CalculateFeaturesCoordinates()
  {

    std::for_each(
        mspPointTemplate.begin(), mspPointTemplate.end(),
        [this](MapPoint *const &it) {
          // Timer timer;
          // timer.start();
          if ((it))
          {
            if (!it->isBad())
            {

              (*it).lastincorporasion = false;
              std::map<KeyFrame *, size_t> obs = (*it).GetObservations();
              // Barycentric coordinates calculated with each KeyFrame
              Eigen::Vector3f MapPoint;

              cv::Mat MapPointPosition = (*it).GetWorldPos();

              MapPoint << MapPointPosition.at<float>(0),
                  MapPointPosition.at<float>(1), MapPointPosition.at<float>(2);

              Node *ClosestNode = static_cast<Node *>(nullptr);
              double bestdist(100);
              for (std::set<Node *>::iterator itNd = mspNodeTemplate.begin();
                   itNd != mspNodeTemplate.end(); itNd++)
              {
                double x, y, z;
                (*itNd)->getXYZ(x, y, z);
                double dist =
                    std::sqrt(pow(x - MapPoint(0), 2) + pow(y - MapPoint(1), 2) +
                              pow(z - MapPoint(2), 2));
                if (dist < bestdist)
                {
                  ClosestNode = (*itNd);
                  bestdist = dist;
                }
              }
              if (ClosestNode)
              {
                std::set<Facet *> FacetsNode = ClosestNode->GetFacets();
                for (std::set<Facet *>::iterator ita = FacetsNode.begin();
                     ita != FacetsNode.end(); ita++)
                {
                  std::set<Node *> node = (*ita)->getNodes();
                  std::vector<Node *> nods;
                  std::vector<Eigen::Vector3f> nodsEigen;
                  for (std::set<Node *>::iterator ite = node.begin();
                       ite != node.end(); ite++)
                  {
                    nods.push_back(*ite);
                    Eigen::Vector3f node;
                    node << (*ite)->x, (*ite)->y, (*ite)->z;
                    nodsEigen.push_back(node);
                  }
                  Eigen::Vector3f barycentric;
                  if (pointInTriangle(MapPoint, nodsEigen[0], nodsEigen[1],
                                      nodsEigen[2], barycentric))
                  {
                    static_cast<DeformationMapPoint *>(it)->SetCoordinates(
                        barycentric(0), barycentric(1), barycentric(2));
                    static_cast<DeformationMapPoint *>(it)->SetFacet(*ita);
                    static_cast<DeformationMapPoint *>(it)->Repose();
                    break;
                  }
                }
              }
            }
          }
        });
  }

  bool TriangularMesh::pointInTriangle(const Eigen::Vector3f &query_point,
                                       const Eigen::Vector3f &triangle_vertex_0,
                                       const Eigen::Vector3f &triangle_vertex_1,
                                       const Eigen::Vector3f &triangle_vertex_2,
                                       Eigen::Vector3f &barycentric)
  {
    // u=P2−P1
    Eigen::Vector3f u = triangle_vertex_1 - triangle_vertex_0;
    // v=P3−P1
    Eigen::Vector3f v = triangle_vertex_2 - triangle_vertex_0;
    // n=u×v
    Eigen::Vector3f n = u.cross(v);
    // w=P−P1
    Eigen::Vector3f w = query_point - triangle_vertex_0;
    // Barycentric coordinates of the projection P′of P onto T:
    // γ=[(u×w)⋅n]/n²
    float gamma = u.cross(w).dot(n) / n.dot(n);
    // β=[(w×v)⋅n]/n²
    float beta = w.cross(v).dot(n) / n.dot(n);
    float alpha = 1 - gamma - beta;
    barycentric << alpha, beta, gamma;
    Eigen::Vector3f newPose = triangle_vertex_0 * alpha +
                              triangle_vertex_1 * beta +
                              triangle_vertex_2 * gamma;
    if ((newPose - query_point).squaredNorm() > 1E-1)
      return false;
    // The point P′ lies inside T if:
    return ((0 <= alpha) && (alpha <= 1) && (0 <= beta) && (beta <= 1) &&
            (0 <= gamma) && (gamma <= 1));
  }

  void TriangularMesh::deletepointsout()
  {
    std::set<MapPoint *> PointsToDelete;
    for (std::set<MapPoint *>::iterator it = mspPointTemplate.begin();
         it != mspPointTemplate.end(); it++)
    {
      if (!static_cast<DeformationMapPoint *>(*it)->getFacet())
      {
        PointsToDelete.insert(*it);
      }
    }
    for (std::set<MapPoint *>::iterator it = PointsToDelete.begin();
         it != PointsToDelete.end(); it++)
    {
      mspPointTemplate.erase(*it);
      static_cast<DeformationMapPoint *>(*it)->SetNoFacet();
    }
  }

  void TriangularMesh::DiscardFaces()
  {
    std::set<Node *> NodesWithObservations;

    for (std::set<Facet *>::iterator itf = mspFacets.begin();
         itf != mspFacets.end(); itf++)
    {
      if (!(*itf)->NoObservations())
      {
        std::array<Node *, 3> Nodes = (*itf)->getNodesArray();
        NodesWithObservations.insert(Nodes[0]);
        NodesWithObservations.insert(Nodes[1]);
        NodesWithObservations.insert(Nodes[2]);
      }
    }
    uint NeighboursToKeep(0);
    std::set<Node *> NodesToKeep =
        CalculateNodesToOptimize(NodesWithObservations, NeighboursToKeep);

    std::set<Facet *> FacetsToKeep;
    for (std::set<Node *>::iterator itn = NodesToKeep.begin();
         itn != NodesToKeep.end(); itn++)
    {
      std::set<Facet *> FacetInNode = (*itn)->GetFacets();
      for (std::set<Facet *>::iterator itf = FacetInNode.begin();
           itf != FacetInNode.end(); itf++)
      {
        FacetsToKeep.insert(*itf);
      }
    }

    std::set<Facet *> OriginalFacets = mspFacets;
    for (std::set<Facet *>::iterator itf = OriginalFacets.begin();
         itf != OriginalFacets.end(); itf++)
    {
      if (FacetsToKeep.end() == FacetsToKeep.find(*itf))
      {
        (*itf)->SetBadFlag();
      }
    }
  }

  void TriangularMesh::generateCoordinates(MapPoint *pMP, cv::Mat &K,
                                           cv::Mat &CamPosition,
                                           cv::KeyPoint &kp)
  {
    Eigen::Vector3d kpe;
    Eigen::Matrix3d Ke;
    cv::cv2eigen(K, Ke);
    kpe << kp.pt.x, kp.pt.y, 1;
    Eigen::Vector3d Rxyz;
    Rxyz = Ke.inverse() * kpe;

    Eigen::Vector4d RxyzC;
    RxyzC << Rxyz, 1;
    // cv::Mat MapPointPosition = (pMP)->GetWorldPos();
    Eigen::Matrix4d Tcw;
    cv::cv2eigen(CamPosition, Tcw);

    Eigen::Vector4d CamMapPoint, cam, MapPoint;
    cam << CamPosition.at<float>(0), CamPosition.at<float>(1),
        CamPosition.at<float>(2), CamPosition.at<float>(3);
    RxyzC = RxyzC / RxyzC(3);
    MapPoint = Tcw.inverse() * RxyzC;

    CamMapPoint = cam - MapPoint;

    for (std::set<Facet *>::iterator ita = mspFacets.begin();
         ita != mspFacets.end(); ita++)
    {
      std::set<Node *> node = (*ita)->getNodes();
      std::vector<Node *> nods;
      for (std::set<Node *>::iterator ite = node.begin(); ite != node.end();
           ite++)
      {
        nods.push_back(*ite);
      }

      Eigen::Matrix4d vABC;

      vABC << nods[0]->x, nods[1]->x, nods[2]->x, CamMapPoint(0), nods[0]->y,
          nods[1]->y, nods[2]->y, CamMapPoint(1), nods[0]->z, nods[1]->z,
          nods[2]->z, CamMapPoint(2), 1, 1, 1, 0;

      Eigen::Vector4d barycentric = vABC.partialPivLu().solve(MapPoint);

      if ((barycentric(0) >= 0 && barycentric(1) >= 0 && barycentric(2) >= 0))
      {
        static_cast<DeformationMapPoint *>(pMP)->SetFacet(*ita);
        static_cast<DeformationMapPoint *>(pMP)->SetCoordinates(
            barycentric(0), barycentric(1), barycentric(2));
        static_cast<DeformationMapPoint *>(pMP)->RecalculatePosition();
        // pMP->SetBadFlag();
      }
    }
  }

  void TriangularMesh::GetFacetTexture()
  {
    ////// Loop the faces to extract the texture
    for (std::set<Facet *>::iterator itf = mspFacets.begin();
         itf != mspFacets.end(); itf++)
    {
      std::set<MapPoint *> FacetMapPoints((*itf)->getMapPoints());
      std::set<KeyFrame *> KeyFramesFacet;

      if (FacetMapPoints.empty())
      {
        std::vector<KeyFrame *> s = (mMap->GetAllKeyFrames());
        KeyFramesFacet = std::set<KeyFrame *>(s.begin(), s.end());
        ;
      }
      for (std::set<MapPoint *>::iterator itfmp = FacetMapPoints.begin();
           itfmp != FacetMapPoints.end(); itfmp++)
      {
        KeyFramesFacet.insert((*itfmp)->GetReferenceKeyFrame());
      }

      (*itf)->setKeyframes(KeyFramesFacet);
      /// Extract texture of the image;
      (*itf)->getTextureCoordinates();
    }
  }

  void TriangularMesh::GetFacetTexture(KeyFrame *KF)
  {
    ////// Loop the faces
    for (std::set<Facet *>::iterator itf = mspFacets.begin();
         itf != mspFacets.end(); itf++)
    {
      /// Extract texture of the image;

      (*itf)->getTextureCoordinates(KF);
    }
  }

  std::set<Node *> CalculateNodesToOptimize(const std::set<Node *> ViewedNodes,
                                            uint NN)
  {
    std::set<Node *> OPtNodes;
    std::set<Node *> NewOptNodes;
    OPtNodes = ViewedNodes;
    int a(0);
    for (uint n(0); n < (NN); n++)
    {
      for (std::set<Node *>::iterator it = OPtNodes.begin(); it != OPtNodes.end();
           it++)
      {
        std::set<Node *> Neighbours = (*it)->GetNeighbours();
        for (std::set<Node *>::iterator ite = Neighbours.begin();
             ite != Neighbours.end(); ite++)
        {
          NewOptNodes.insert(*ite);
          a++;
        }
      }
      for (std::set<Node *>::iterator it = NewOptNodes.begin();
           it != NewOptNodes.end(); it++)
      {
        OPtNodes.insert(*it);
      }
    }
    return OPtNodes;
  }
} // namespace defSLAM
