#include "PoissonReconstruction.h"
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/impl/search.hpp>

using namespace pcl;
using namespace std;

PoissonReconstruction::PoissonReconstruction(std::vector<float> & Points, double Radius, int pointDensity, int KSearch, int depth)
    :Radius (Radius), pointDensity (pointDensity),  KSearch(KSearch), depth (depth){
    points = std::vector<float>(Points.begin(),Points.end());

}

void PoissonReconstruction::execute(std::vector<std::vector<float> > &nodes, std::vector<std::vector<int>> & facets){
       PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());

       for (uint i(0);i<points.size();i=i+3)
       {
               pcl::PointXYZ pclpoint;
               pclpoint.x = points[i];
               pclpoint.y = points[i+1];
               pclpoint.z = points[i+2];
               cloud->push_back(pclpoint);
      }


        MovingLeastSquares<PointXYZ, PointXYZ> mls;
        mls.setInputCloud (cloud);
        mls.setSearchRadius (Radius);
        mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::RANDOM_UNIFORM_DENSITY);
        mls.setPointDensity(pointDensity);

       // mls.setUpsamplingStepSize (0.007);
        PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ> ());
        mls.process (*cloud_smoothed);


       pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
          pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
          n.setNumberOfThreads (2);
          tree->setInputCloud (cloud);
          n.setInputCloud (cloud);
          n.setSearchMethod (tree);
          n.setKSearch (KSearch);

          n.setViewPoint(0,0,0);
          pcl::PointCloud<pcl::Normal> normals;
          normals.reserve(points.size()/3);
          n.compute (normals);
          //* normals should not contain the point normals + surface curvatures
          // Concatenate the XYZ and normal fields*
          PointCloud<PointNormal>::Ptr cloud_smoothed_normals (new PointCloud<PointNormal> ());
          pcl::concatenateFields (*cloud, normals, *cloud_smoothed_normals);

       //std::cout << "Point cloud after mls for template with: " << cloud->size() << " Points" << std::endl;
       pcl::PCLPointCloud2::Ptr cloud_smoothed22 (new pcl::PCLPointCloud2 ());

       pcl::toPCLPointCloud2(*cloud,*cloud_smoothed22);
       pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
       pcl::toPCLPointCloud2(*cloud,*cloud2);
       Poisson<PointNormal> poisson;

       /// Usually the Depth value is 9, but as this point cloud is sparse we use 4
       poisson.setDepth(depth);
       poisson.setDegree(2);
       poisson.setSolverDivide (9);
       poisson.setInputCloud
               (cloud_smoothed_normals);
       PolygonMesh mesh;

       poisson.reconstruct (mesh);

       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2a (new pcl::PointCloud<pcl::PointXYZ>);
       pcl::fromPCLPointCloud2 (mesh.cloud, *cloud2a);
       nodes.reserve(cloud2a->points.size());
       for (uint j=0; j < cloud2a->points.size(); j++){
          std::vector<float> node;
          node.reserve(3);
          node.push_back(cloud2a->points[j].x);
          node.push_back(cloud2a->points[j].y);
          node.push_back(cloud2a->points[j].z);
          nodes.push_back(node);
       }
       facets.reserve(mesh.polygons.size());
       for (uint j=0; j < mesh.polygons.size(); j++){
           std::vector<int> facet;
           facet.reserve(3);
           for(uint i(0);i<3;i++){
                facet.push_back(mesh.polygons[j].vertices[i]);
           }
           facets.push_back(facet);
        }
       cout << "Poisson reconstruction " << mesh.polygons.size() << " Polygons"<<   endl;
       //pcl::io::saveVTKFile ("meshPoisson.vtk", mesh);
}
