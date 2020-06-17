#include "LaplacianMesh.h"

namespace defSLAM {
using ORB_SLAM2::Map;
using ORB_SLAM2::MapPoint;

LaplacianMesh::LaplacianMesh(std::set<MapPoint *> &mspMapPoints, Map *map)
    : TriangularMesh(mspMapPoints, map), maxCurv(0.0) {
  this->ExtractMeanCurvatures();
}

LaplacianMesh::LaplacianMesh(std::vector<std::vector<double>> &vertex,
                             std::vector<std::vector<int>> &index, Map *map)
    : TriangularMesh(vertex, index, map), maxCurv(0.0) {
  this->ExtractMeanCurvatures();
}

LaplacianMesh::LaplacianMesh(std::set<MapPoint *> &mspMapPoints, Map *map,
                             KeyFrame *kF)
    : TriangularMesh(mspMapPoints, map, kF), maxCurv(0.0) {
  this->ExtractMeanCurvatures();
}

LaplacianMesh::~LaplacianMesh() { LaplacianCoords.clear(); }

void LaplacianMesh::ExtractMeanCurvatures() {
  for (std::set<Node *>::iterator it = mspNodeTemplate.begin();
       it != mspNodeTemplate.end(); it++) {
    std::set<Node *> Neighbours = (*it)->GetNeighbours();
    Eigen::Vector3d Ni;
    Ni << (*it)->x, (*it)->y, (*it)->z;

    Eigen::Vector3d Laplacian;
    Laplacian.Zero();
    Eigen::Vector3d NeightMean;
    NeightMean.Zero();

    // Voronoi region of Ring of the node i
    for (std::set<Node *>::iterator ite = Neighbours.begin();
         ite != Neighbours.end(); ite++) {
      std::set<Node *> NeighboursofN = (*ite)->GetNeighbours();
      Eigen::Vector3d Nj;
      Nj << (*ite)->x, (*ite)->y, (*ite)->z;

      // To discover the neighbours j+1 and j-1 of j
      std::vector<Node *> Nj1j_1;
      for (std::set<Node *>::iterator itn = NeighboursofN.begin();
           itn != NeighboursofN.end(); itn++) {
        if (Neighbours.count(*itn)) {
          Nj1j_1.push_back(*itn);
        }
      }

      if (Nj1j_1.size() == 0) {
        (*ite)->SetBadFlag();
      } else if (Nj1j_1.size() == 1) {
        (*ite)->setBoundary();
      } else {
        (*it)->NodesjJ_1J[*ite] =
            (std::pair<Node *, Node *>(Nj1j_1[0], Nj1j_1[1]));
        Eigen::Vector3d Nj1;
        Nj1 << (Nj1j_1[0])->x, (Nj1j_1[0])->y, (Nj1j_1[0])->z;

        Eigen::Vector3d Nj_1;
        Nj_1 << (Nj1j_1[1])->x, (Nj1j_1[1])->y, (Nj1j_1[1])->z;

        Eigen::Vector1d tnomega1;
        tnomega1 << ((Nj_1 - Ni).cross(Nj - Ni)).norm() /
                        ((Nj_1 - Ni).dot(Nj - Ni));
        Eigen::Vector1d tnomega2;
        tnomega2 << ((Nj1 - Ni).cross(Nj - Ni)).norm() /
                        ((Nj1 - Ni).dot(Nj - Ni));
        double wij = (tan(std::abs(atan(tnomega1(0))) / 2) +
                      tan(std::abs(atan(tnomega2(0))) / 2)) /
                     (Ni - Nj).norm();

        (*it)->weights[*ite] = (wij);
      }
    }
  }

  for (std::set<Node *>::iterator it = mspNodeTemplate.begin();
       it != mspNodeTemplate.end(); it++) {
    if (!(*it)->isBoundary()) {
      std::set<Node *> Neighbours = (*it)->GetNeighbours();
      if (Neighbours.size() > 1) {
        Eigen::Vector3d Laplacian;
        Laplacian.Zero();
        Laplacian << 0, 0, 0;
        Eigen::Vector3d Ni;
        Ni << (*it)->x, (*it)->y, (*it)->z;
        double sumweights(0.0);
        std::set<Node *> Neighbours = (*it)->GetNeighbours();
        for (std::set<Node *>::iterator ite = Neighbours.begin();
             ite != Neighbours.end(); ite++) {
          Eigen::Vector3d Nj;
          Nj << (*ite)->x, (*ite)->y, (*ite)->z;
          Laplacian.noalias() = Laplacian + (*it)->weights[(*ite)] * (Nj);
          sumweights = sumweights + (*it)->weights[(*ite)];
        }

        LaplacianCoords[*it] = (Ni - (Laplacian / sumweights)).transpose();
        if ((Laplacian / sumweights).norm() > maxCurv)
          maxCurv = (Laplacian / sumweights).norm();
      }
    }
  }
}

Eigen::Vector3d const LaplacianMesh::GetLaplacianCoord(Node *n) {
  return LaplacianCoords.at(n);
}

const Eigen::Vector1d LaplacianMesh::GetMeanCurvatureInitial(Node *n) {
  Eigen::Vector1d mc;
  mc << LaplacianCoords.at(n).norm();
  return mc;
}

Eigen::Vector1d const LaplacianMesh::GetMeanCurvature(Node *n) {
  Eigen::Vector3d Laplacian;
  Laplacian.Zero();
  Laplacian << 0, 0, 0;
  Eigen::Vector3d Ni;
  Ni << (n)->x, (n)->y, (n)->z;
  double sumweights(0.0);
  std::set<Node *> Neighbours = (n)->GetNeighbours();
  for (std::set<Node *>::iterator ite = Neighbours.begin();
       ite != Neighbours.end(); ite++) {
    Eigen::Vector3d Nj;
    Nj << (*ite)->x, (*ite)->y, (*ite)->z;

    Laplacian.noalias() = Laplacian + (n)->weights[(*ite)] * (Nj);
    sumweights =
        sumweights +
        (n)->weights[(
            *ite)]; //(1.0/8.0)*(*it)->weights[(*ite)]*(Nj-Ni).dot((Nj-Ni));
  }
  Eigen::Vector1d mc;
  mc << (Laplacian.norm() / sumweights); // LaplacianCoords.at(n).norm();

  return mc;
}

Eigen::Vector3d const LaplacianMesh::GetMeanCurvatureVector(Node *n) {
  Eigen::Vector3d Laplacian;
  Laplacian.Zero();
  Laplacian << 0, 0, 0;
  Eigen::Vector3d Ni;
  Ni << (n)->x, (n)->y, (n)->z;
  double sumweights(0.0);
  std::set<Node *> Neighbours = (n)->GetNeighbours();
  for (std::set<Node *>::iterator ite = Neighbours.begin();
       ite != Neighbours.end(); ite++) {
    Eigen::Vector3d Nj;
    Nj << (*ite)->x, (*ite)->y, (*ite)->z;

    Laplacian.noalias() = Laplacian + (n)->weights[(*ite)] * (Nj);
    sumweights = sumweights + (n)->weights[(*ite)];
  }

  Eigen::Vector1d mc;
  mc << (Laplacian.norm() / sumweights);

  return Laplacian / sumweights;
}

} // namespace ORB_SLAM2
