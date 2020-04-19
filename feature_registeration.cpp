//
// Created by haeyeon on 20. 4. 11..
//

#include "feature_registeration.h"
#include "visualize_correspondences.h"
#include <pcl/filters/voxel_grid.h>



void addNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
               pcl::PointCloud<pcl::Normal>::Ptr normals,
               pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals
)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZ>);
    searchTree->setInputCloud ( cloud );

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud ( cloud );
    normalEstimator.setSearchMethod ( searchTree );
    normalEstimator.setKSearch ( 15 );
    normalEstimator.compute ( *normals );

    pcl::concatenateFields( *cloud, *normals, *cloud_with_normals );
}




void
loadFile(const char* fileName,
         pcl::PointCloud<pcl::PointXYZ> &cloud
)
{
    pcl::PolygonMesh mesh;

    if ( pcl::io::loadPolygonFile ( fileName, mesh ) == -1 )
    {
        PCL_ERROR ( "loadFile faild." );
        return;
    }
    else
        pcl::fromPCLPointCloud2<pcl::PointXYZ> ( mesh.cloud, cloud );

    // remove points having values of nan
    std::vector<int> index;
    pcl::removeNaNFromPointCloud ( cloud, cloud, index );
}
//
//int main ()
//{
//
//    pcl::PCLPointCloud2::Ptr source_cloud (new pcl::PCLPointCloud2 ());
//    pcl::PCLPointCloud2::Ptr source_filtered (new pcl::PCLPointCloud2 ());
//    pcl::PCLPointCloud2::Ptr target_cloud (new pcl::PCLPointCloud2 ());
//    pcl::PCLPointCloud2::Ptr target_filtered (new pcl::PCLPointCloud2 ());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
//
//    std::string src_filename = "/home/haeyeon/lidar_icp/data/data2/1.pcd";
//    std::string trg_filename = "/home/haeyeon/lidar_icp/data/data2/50.pcd";
//    pcl::PCDReader reader;
//    reader.read(src_filename, *source_cloud);
//    reader.read(trg_filename, *target_cloud);
//    // downsampling
//    pcl::PCLPointCloud2::Ptr cloudVoxel (new pcl::PCLPointCloud2 ());
//    pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;
//    voxelGrid.setInputCloud(source_cloud);
//    voxelGrid.setLeafSize(0.1, 0.1, 0.1);
//    voxelGrid.filter(*source_filtered);
//
//    voxelGrid.setInputCloud(target_cloud);
//    voxelGrid.setLeafSize(0.1, 0.1, 0.1);
//    voxelGrid.filter(*target_filtered);
//
//    pcl::fromPCLPointCloud2(*source_filtered, *source);
//    pcl::fromPCLPointCloud2(*target_filtered, *target);
//
////    {
////        // load source
////        loadFile ( argv[1], *cloud_source );
////        // load target
////        loadFile ( argv[2], *cloud_target );
////    }
//
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans ( new pcl::PointCloud<pcl::PointXYZ> () );
//
//    // prepare could with normals
//    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
//    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
//    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_trans_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
//
//    pcl::PointCloud<pcl::Normal>::Ptr source_normals ( new pcl::PointCloud<pcl::Normal> () );
//    pcl::PointCloud<pcl::Normal>::Ptr target_normals ( new pcl::PointCloud<pcl::Normal> () );
//
//    addNormal ( source, source_normals, cloud_source_normals );
//    addNormal ( target, target_normals, cloud_target_normals );
//    addNormal ( source, source_normals, cloud_source_trans_normals ); // dummy at this time
//
//
//
//
//
//
//    float radius;
//    {
//        Eigen::Vector4f max_pt, min_pt;
//        pcl::getMinMax3D ( *source, min_pt, max_pt );
//        radius = (max_pt - min_pt).norm() / 50; // fixed radius (scale) for detector/descriptor
//        std::cout << "scale: " << radius << std::endl;
//    }
//
//
//
//    { // registration
//
//        pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints ( new pcl::PointCloud<pcl::PointXYZI> () );
//        pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints ( new pcl::PointCloud<pcl::PointXYZI> () );
//        pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ ( new pcl::PointCloud<pcl::PointXYZ> () );
//        pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ ( new pcl::PointCloud<pcl::PointXYZ> () );
//
//
//        { // Harris detector with normal
//            std::cout << "detection" << std::endl;
//
//            pcl::HarrisKeypoint3D<pcl::PointXYZRGBNormal,pcl::PointXYZI>::Ptr detector ( new pcl::HarrisKeypoint3D<pcl::PointXYZRGBNormal,pcl::PointXYZI> () );
//            detector->setNonMaxSupression ( true );
//            detector->setRadius ( radius );
//            detector->setRadiusSearch ( radius );
//            detector->setMethod ( pcl::HarrisKeypoint3D<pcl::PointXYZRGBNormal,pcl::PointXYZI>::CURVATURE ); // HARRIS, NOBLE, LOWE, TOMASI, CURVATURE
//
//            detector->setInputCloud ( cloud_source_normals );
//            detector->setSearchSurface ( cloud_source_normals );
//            detector->compute ( *source_keypoints );
//            cout << "number of source keypoints found: " << source_keypoints->points.size() << endl;
//            source_keypointsXYZ->points.resize( source_keypoints->points.size() );
//            pcl::copyPointCloud( *source_keypoints, *source_keypointsXYZ );
//
//            detector->setInputCloud ( cloud_target_normals );
//            detector->setSearchSurface ( cloud_target_normals );
//            detector->compute ( *target_keypoints );
//            cout << "number of target keypoints found: " << target_keypoints->points.size() << endl;
//            source_keypointsXYZ->points.resize( target_keypoints->points.size() );
//            pcl::copyPointCloud( *target_keypoints, *target_keypointsXYZ );
//
//        }
//
//
//
//#define useFPFH
//
//#ifdef useFPFH
//#define descriptorType pcl::FPFHSignature33
//#else
//#define descriptorType pcl::SHOT352
//#endif
//
//        pcl::PointCloud<descriptorType>::Ptr source_features ( new pcl::PointCloud<descriptorType> () );
//        pcl::PointCloud<descriptorType>::Ptr target_features ( new pcl::PointCloud<descriptorType> () );
//
//        { // descriptor
//            std::cout << "description" << std::endl;
//#ifdef useFPFH
//            pcl::Feature<pcl::PointXYZ, descriptorType>::Ptr descriptor ( new pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, descriptorType> () );
//#else
//            pcl::Feature<pcl::PointXYZ, descriptorType>::Ptr descriptor ( new pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, descriptorType> () );
//#endif
//            descriptor->setSearchMethod ( pcl::search::Search<pcl::PointXYZ>::Ptr ( new pcl::search::KdTree<pcl::PointXYZ>) );
//            descriptor->setRadiusSearch ( radius*10 );
//
//            pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, descriptorType>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, descriptorType> > ( descriptor );
//
//            descriptor->setSearchSurface ( source );
//            descriptor->setInputCloud ( source_keypointsXYZ );
//            feature_from_normals->setInputNormals ( source_normals );
//            descriptor->compute ( *source_features );
//
//
//            descriptor->setSearchSurface ( target );
//            descriptor->setInputCloud ( target_keypointsXYZ );
//            feature_from_normals->setInputNormals ( target_normals );
//            descriptor->compute ( *target_features );
//
//        }
//
//
//
//        std::vector<int> correspondences;
//
//        { // Find matching with Kd-tree
//            std::cout << "matching" << std::endl;
//
//            correspondences.resize ( source_features->size() );
//
//            pcl::KdTreeFLANN<descriptorType> search_tree;
//            search_tree.setInputCloud ( target_features );
//
//            std::vector<int> index(1);
//            std::vector<float> L2_distance(1);
//            for (int i = 0; i < source_features->size(); ++i)
//            {
//                correspondences[i] = -1; // -1 means no correspondence
//
//#ifdef useFPFH
//                if ( isnan ( source_features->points[i].histogram[0] ) ) continue;
//#else
//                if ( isnan ( source_features->points[i].descriptor[0] ) ) continue;
//#endif
//
//                search_tree.nearestKSearch ( *source_features, i, 1, index, L2_distance );
//                correspondences[i] = index[0];
//
//            }
//
//        }
//        visualize_correspondences (source, source_keypointsXYZ,
//                                   target, target_keypointsXYZ,
//                                   correspondences);
//
//
//        pcl::CorrespondencesPtr pCorrespondences ( new pcl::Correspondences );
//
//        { // Refining matching by filtering out wrong correspondence
//            std::cout << "refineing matching" << std::endl;
//
//            int nCorrespondence = 0;
//            for (int i = 0; i < correspondences.size(); i++)
//                if ( correspondences[i] >= 0 ) nCorrespondence++; // do not count "-1" in correspondences
//
//            pCorrespondences->resize ( nCorrespondence );
//            for (int i = 0, j = 0; i < correspondences.size(); i++)
//            {
//                if ( correspondences[i] > 0 )
//                {
//                    (*pCorrespondences)[j].index_query = i;
//                    (*pCorrespondences)[j].index_match = correspondences[i];
//                    j++;
//
//                }
//            }
//
//            pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> refine;
//            refine.setInputSource ( source_keypointsXYZ );
//            refine.setInputTarget ( target_keypointsXYZ );
//            refine.setInputCorrespondences ( pCorrespondences );
//            refine.getCorrespondences ( *pCorrespondences );
//        }
//        visualize_correspondences (source, source_keypointsXYZ,
//                                   target, target_keypointsXYZ,
//                                   pCorrespondences);
//
//
//
//
//        Eigen::Matrix4f transformation;
//
//        { // Estimating rigid transformation
//            std::cout << "Estimating transformation" << std::endl;
//
//            pcl::registration::TransformationEstimationSVD< pcl::PointXYZ, pcl::PointXYZ > est;
//
//            std::vector<int> source_index ( source_features->size() );
//            for (int i = 0; i < source_features->size(); ++i) source_index[i] = i;
//
//
//            est.estimateRigidTransformation ( *source_keypointsXYZ,
//                                              *target_keypointsXYZ, *pCorrespondences,
//                                              transformation );
//            std::cout << transformation << std::endl;
//
//            pcl::transformPointCloud ( *source, *cloud_source_trans, transformation );
//            pcl::transformPointCloud ( *cloud_source_normals, *cloud_source_trans_normals, transformation );
//
//        }
//
//
//    }
//
//
//
//    {
//        // visualization
//        boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
//        viewer->setBackgroundColor (0, 0, 0);
//
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color ( source, 0, 255, 0 );
//        viewer->addPointCloud<pcl::PointXYZ> (source, source_color, "source");
//        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");
//
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color ( target, 255, 255, 255 );
//        viewer->addPointCloud<pcl::PointXYZ> ( target, target_color, "target");
//        viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target" );
//
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_trans_color ( cloud_source_trans, 255, 0, 255 );
//        viewer->addPointCloud<pcl::PointXYZ> ( cloud_source_trans, source_trans_color, "source trans" );
//        viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source trans" );
//
//
//        // orthographic (parallel) projection; same with pressing key 'o'
//        viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection( 1 );
//
//        viewer->resetCamera();
//
//        viewer->spin ();
//
//
//
//
//        { // addtional refinement wit ICP
//            pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
//
//            icp.setInputSource ( cloud_source_trans_normals );
//            icp.setInputTarget ( cloud_target_normals );
//
//            // registration
//            icp.align ( *cloud_source_trans_normals );
//
//            Eigen::Matrix4f transformation;
//            if ( icp.hasConverged() )
//            {
//                pcl::transformPointCloud ( *cloud_source_trans, *cloud_source_trans, icp.getFinalTransformation() );
//                viewer->updatePointCloud ( cloud_source_trans, source_trans_color, "source trans" );
//            }
//            else
//                std::cout << "Not converged." << std::endl;
//
//            viewer->spin ();
//        }
//    }
//
//
//    return 0;
//}
