#include "point_registration.h"

int
point_registration ()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");

    std::string filename = "/home/haeyeon/lidar_icp/data/data1/1.pcd";
    pcl::PCDReader reader;
    reader.read(filename, *cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    pcl::copyPointCloud (*cloud, *inliers, *final);
    viewer.addPointCloud(final,"cloud");
    viewer.spin();
    return 0;
}