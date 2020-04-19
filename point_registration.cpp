#include <pcl/filters/extract_indices.h>
#include "point_registration.h"

int
//point_registration ()
main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    std::string filepath = "/home/haeyeon/data/data1/";

    for( int i=1; i<744; i ++) {
        std::string filename = filepath + to_string(i) + ".pcd";
        reader.read(filename, *cloud);

        // extract vertical walls
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.4);
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*cloud);

        // find nearest point from the origin
        pcl::PointXYZ origin (0,0,0);
        pcl::KdTree<pcl::PointXYZ>::Ptr tree_ (new pcl::KdTreeFLANN<pcl::PointXYZ>);
        tree_->setInputCloud(cloud);

        std::vector<int> nn_indices (1);
        std::vector<float> nn_dists (1);

        tree_->nearestKSearch(origin, 1, nn_indices, nn_dists);
        printf("The closest point: (%f, %f, %f), distance: %f \n", cloud->points[nn_indices[0]].x, cloud->points[nn_indices[0]].y, cloud->points[nn_indices[0]].z, nn_dists[0]);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
        viewer.removeAllPointClouds();
        viewer.removeAllShapes();
        viewer.addPointCloud(cloud, single_color,"cloud"+to_string(i));
        viewer.addLine (origin, cloud->points[nn_indices[0]],255,0,0, to_string(i));
        viewer.spinOnce();
        sleep(0.5);
    }
    viewer.spin();

    return 0;
}