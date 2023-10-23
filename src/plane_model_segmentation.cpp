#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

ros::Publisher pub_output;
ros::Publisher pub_coefficients;

float k_search = 30;
float normal_distance_weight = 0.1;
float distance_threshould = 0.1;
float max_iterations = 1000;
bool extract_negative = false;

void
cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  ros::Time start = ros::Time::now();

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  // Estimate point normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (k_search);
  ne.compute (*cloud_normals);

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (normal_distance_weight);
  seg.setDistanceThreshold (distance_threshould);
  seg.setMaxIterations (max_iterations);
  seg.setInputCloud (cloud);//pcl::pclPointcloud2Ptrには対応していない
  seg.setInputNormals (cloud_normals);
  seg.segment (*inliers, *coefficients);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_inliers->header = cloud->header;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setNegative (extract_negative);
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.filter (*cloud_inliers);

  pcl_conversions::toPCL(ros::Time::now(), cloud_inliers->header.stamp);
  pub_output.publish(cloud_inliers);

  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(*coefficients, ros_coefficients);
  pub_coefficients.publish (ros_coefficients);

  ros::Duration duration = ros::Time::now() - start;
  ROS_INFO("Duration %.5ld[nsec]", duration.toNSec());
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "range_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("k_search", k_search);
  pnh.getParam("distance_threshould", distance_threshould);
  pnh.getParam("normal_distance_weight", normal_distance_weight);
  pnh.getParam("max_iterations", max_iterations);
  pnh.getParam("extract_negative", extract_negative);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_output = nh.advertise<pcl::PCLPointCloud2> ("output", 1);
  pub_coefficients = nh.advertise<pcl_msgs::ModelCoefficients> ("model_coefficients", 1);

  // Spin
  ros::spin ();
}