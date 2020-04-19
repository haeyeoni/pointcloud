//
// Created by haeyeon on 20. 4. 11..
//
#include <iostream>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

#ifndef LIDAR_ICP_HELLO_WORLD_H
#define LIDAR_ICP_HELLO_WORLD_H
int point_registration();
#endif //LIDAR_ICP_HELLO_WORLD_H
