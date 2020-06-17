#include <SmootherMLS.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/common/distances.h>

#include <pcl/filters/radius_outlier_removal.h>

SmootherMLS::SmootherMLS(int polynomialOrder_ ,double searchRadius_)
:polynomialOrder_(polynomialOrder_),searchRadius_(searchRadius_)
{

};

using namespace pcl;

std::vector<int> SmootherMLS::smoothPointCloud(const SmootherMLS::pointcloud& pcVector){
    pcl::PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ> ());

    for (uint i(0);i<pcVector.size();i++)
    {
               PointXYZ pclpoint;
               pclpoint.x = pcVector[i][0];
               pclpoint.y = pcVector[i][1];
               pclpoint.z = pcVector[i][2];
               cloud->push_back(pclpoint);
    }
    std::cout << "initial.size()" << pcVector.size() <<std::endl;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal> mls_points;
    // Set parameters
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setInputCloud (cloud);
    mls.setComputeNormals(true); // To have the corresponding Indices
    mls.setPolynomialOrder (polynomialOrder_);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (searchRadius_);
    mls.process (mls_points);
    auto indices = (*(mls.getCorrespondingIndices())).indices;
    SmootherMLS::pointcloud pcVectorO;
    pcVectorO.reserve(mls_points.size());
 
    for (uint i(0);i<pcVector.size();i++){
        std::vector<float> pt;
        pt.reserve(3);
        pt.push_back(mls_points[i].x);
        pt.push_back(mls_points[i].y);
        pt.push_back(mls_points[i].z);
        pcVectorO.push_back(std::move(pt));
    }
    std::cout << "mls_points.size()" << mls_points.size() <<std::endl;
    std::cout << "mls_points.size()" << indices.size() <<std::endl;

    return indices;
}


std::vector<int> SmootherMLS::outlierRemovalRadius(const SmootherMLS::pointcloud& pcVector){
    pcl::PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ> ());

    for (uint i(0);i<pcVector.size();i++)
    {
               PointXYZ pclpoint;
               pclpoint.x = pcVector[i][0];
               pclpoint.y = pcVector[i][1];
               pclpoint.z = pcVector[i][2];
               cloud->push_back(pclpoint);
    }
    std::cout << "initial.size()" << pcVector.size() <<std::endl;

    // Set parameters
   pcl::RadiusOutlierRemoval<PointXYZ> outrem;
   outrem.setInputCloud(cloud);
   outrem.setRadiusSearch(searchRadius_);
   outrem.setMinNeighborsInRadius (20);
   std::vector<int> indices;
   outrem.filter (indices);
   return indices;
}