//
// Created by haeyeon on 20. 4. 11..
//

#ifndef LIDAR_ICP_FEATURE_REGISTERATION_H
#define LIDAR_ICP_FEATURE_REGISTERATION_H


#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>

#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <algorithm>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

class feature_registeration {

};


#endif //LIDAR_ICP_FEATURE_REGISTERATION_H
