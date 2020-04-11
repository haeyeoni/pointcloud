//
// Created by haeyeon on 20. 4. 11..
//

#ifndef LIDAR_ICP_VISUALIZE_CORRESPONDENCES_H
#define LIDAR_ICP_VISUALIZE_CORRESPONDENCES_H


#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ PointT;
void
visualize_correspondences (const PointCloudPtr points1, const PointCloudPtr keypoints1,
                           const PointCloudPtr points2, const PointCloudPtr keypoints2,
                           const std::vector<int> &correspondences
);
void
visualize_correspondences (const PointCloudPtr points1, const PointCloudPtr keypoints1,
                           const PointCloudPtr points2, const PointCloudPtr keypoints2,
                           const pcl::CorrespondencesPtr pCorrespondences
);
#endif //LIDAR_ICP_VISUALIZE_CORRESPONDENCES_H
