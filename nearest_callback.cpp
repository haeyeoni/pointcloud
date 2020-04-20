#include "nearest_callback.h"
#include <ros/ros.h>
#include <sstream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float64MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
//#include <pcl_ros/transforms.h>
//#include "std_msgs/MultiArrayLayout.h"
//#include "std_msgs/MultiArrayDimension.h"

class SearchNearest
{
public:
    SearchNearest() {
        this->subscriber = this->nh.subscribe("/velodyne_points", 10, &SearchNearest::SubVelodyne, this);
        this->publisher = this->nh.advertise<sensor_msgs::PointCloud> ("nearest_points", 1);
        this->publisher2 = this->nh.advertise<std_msgs::Float64MultiArray> ("distances", 1);
    };

    void SubVelodyne(const sensor_msgs::PointCloud2 &cloud_msg) {
        sensor_msgs::PointCloud2 nearest_points;
        std_msgs::Float64MultiArray distances;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
        pcl::PointCloud<pcl::PointXYZ> result;
        pcl_conversions::toPCL(cloud_msg, *cloud2);
        pcl::fromPCLPointCloud2(*cloud2, *cloud);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // extract vertical walls
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.4);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud);

        // find nearest point from the origin
        pcl::PointXYZ origin(0, 0, 0);
        pcl::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        tree_->setInputCloud(cloud);

        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);
        tree_->nearestKSearch(origin, 1, nn_indices, nn_dists);
        distances.data.clear();
        for (int i = 0; i < nn_indices.size(); i++) {
            result.push_back(cloud->points[nn_indices[i]]);
            distances.data.push_back(nn_dists[i]);
        }
        this->publisher.publish(nearest_points);
        this->publisher2.publish(distances);
    }

    private:
        ros::NodeHandle nh;
        ros::Subscriber subscriber;
        ros::Publisher publisher;
        ros::Publisher publisher2;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    SearchNearest SearchNearest;
    while(1) {
        ros::spinOnce();
    }
}