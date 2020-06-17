
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/feature.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/common/angles.h>
#include <iostream>
#include <pcl/common/distances.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/PolygonMesh.h>
#include <pcl/geometry/triangle_mesh.h>

#include <pcl/surface/reconstruction.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include "opencv2/opencv.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

int main (){
    // Load input file into a PointCloud<T> with an appropriate type
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
     // Load bun0.pcd -- should be available with the PCL archive in test
     pcl::io::loadPCDFile("pointPoisson.pcd", *cloud);

     using namespace pcl;
    using namespace std;


    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal> ());


    // pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
    //pcl::toPCLPointCloud2(*cloud,*cloud2);
    // Create the filtering object
    std::cout << "Point cloud before filter for template with: " << cloud->size() << " Points" << std::endl;

    pcl::PointCloud<pcl::PointNormal> mls_points;

     MovingLeastSquares<PointXYZ, PointXYZ> mls;
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr treea (new pcl::search::KdTree<pcl::PointXYZ>);
    // treea->setInputCloud (cloud);
     mls.setInputCloud (cloud);
     mls.setSearchRadius (0.350);
    // mls.setSearchMethod(treea);
     mls.setPolynomialFit (true);
     mls.setPolynomialOrder (3);
     mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ,  PointXYZ>::RANDOM_UNIFORM_DENSITY);
     mls.setPointDensity(2000);
   //  mls.process(mls_points);
    // mls.setUpsamplingStepSize (0.007);
     PointCloud<PointXYZ>::Ptr cloud_smoothed2 (new PointCloud<PointXYZ> ());
     mls.process (*cloud_smoothed2);
     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
       sor.setInputCloud (cloud_smoothed2);
       sor.setMeanK (cloud->size()/8);
       sor.setStddevMulThresh (0.750);
       PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ> ());

       sor.filter (*cloud_smoothed);
       std::cout << "Point cloud after mls for template with: " << cloud_smoothed->size() << " Points" << std::endl;
       pcl::PCLPointCloud2::Ptr cloud_smoothed22 (new pcl::PCLPointCloud2 ());

       pcl::toPCLPointCloud2(*cloud_smoothed,*cloud_smoothed22);
       pcl::io::saveVTKFile ("pointPoissonSmoothed.vtk", *cloud_smoothed22);



/*
       MovingLeastSquares<PointXYZ, PointXYZ> mls2;
      // pcl::search::KdTree<pcl::PointXYZ>::Ptr treea (new pcl::search::KdTree<pcl::PointXYZ>);
     //  treea->setInputCloud (cloud);
       mls2.setInputCloud (cloud_smoothed2);
       mls2.setSearchRadius (0.50);
      // mls.setSearchMethod(treea);
       mls2.setPolynomialFit (true);
       mls2.setPolynomialOrder (3);
       mls2.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::RANDOM_UNIFORM_DENSITY);
       mls2.setPointDensity(6000);
       mls2.process (*cloud_smoothed);
       std::cout << "Point cloud after mls for template with: " << cloud_smoothed->size() << " Points" << std::endl;
*/
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
       pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
       n.setNumberOfThreads (8);
       tree->setInputCloud (cloud_smoothed);
       n.setInputCloud (cloud_smoothed);
       n.setSearchMethod (tree);
       n.setViewPoint(10,10,10);
     // n.setSearchSurface(cloud_smoothed);
       n.setKSearch (1000);
       n.compute (*normals);
       //* normals should not contain the point normals + surface curvatures
       // Concatenate the XYZ and normal fields*
       PointCloud<PointNormal>::Ptr cloud_smoothed_normals (new PointCloud<PointNormal> ());
       pcl::concatenateFields (*cloud_smoothed, *normals, *cloud_smoothed_normals);
       boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 40, "normals");

  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_smoothed, normals, 1, 0.05, "normals");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (cloud, 255, 125, 0);

    viewer->addPointCloud<pcl::PointXYZ> (cloud_smoothed, single_color, "sample cloud");
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    viewer->close();


    pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
    pcl::toPCLPointCloud2(*cloud,*cloud2);
    pcl::io::saveVTKFile ("pointPoisson.vtk", *cloud2);
    cout << "begin poisson reconstruction" << endl;
    Poisson<PointNormal> poisson;
   // poisson.setManifold(true);
    //poisson.setSamplesPerNode(cloud_smoothed->size()/cloud->size());

  //  PointCloud<PointNormal>::Ptr cloud_smoothed_normals (new PointCloud<PointNormal> (mls_points));

    /// Usually the Depth value is 9, but as this point cloud is sparse we use 4
    poisson.setDepth(5);
    poisson.setDegree(2);
    poisson.setSolverDivide (9);
    poisson.setInputCloud(cloud_smoothed_normals
            );
    PolygonMesh mesh;
    poisson.reconstruct (mesh);
    cout << "Poisson reconstruction " << mesh.polygons.size() << " Polygons"<<   endl;
    pcl::io::saveVTKFile ("meshPoisson.vtk", mesh);
    return 0;
}

