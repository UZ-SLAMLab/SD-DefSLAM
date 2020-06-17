#include "PCLNormalEstimator.h"

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

PCLNormalEstimator::PCLNormalEstimator(std::vector<float> & Points, double radious)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    for(size_t i = 0;i<Points.size();i =i+3){
	cloud->push_back(pcl::PointXYZ(Points[i],Points[i+1],Points[i+2]));
    }

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (radious);
    ne.setViewPoint(0,0,0);

    ne.compute (*cloud_normals);

    for (uint i(0);i<cloud_normals->size();i++){
    		normals.push_back(cloud_normals->at(i).normal_x);
    		normals.push_back(cloud_normals->at(i).normal_y);
    		normals.push_back(cloud_normals->at(i).normal_z);
    }
}

std::vector<float> && PCLNormalEstimator::getNormals(){
     return std::move(normals);
}
