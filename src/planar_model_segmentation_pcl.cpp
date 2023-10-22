#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>

ros::Publisher pub_coefficients;
ros::Publisher pub_output;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  // pcl::SACSegmentation<pcl::PointXYZ> seg;
  // pcl::SACSegmentationFromNormals<pcl::PointXYZ> seg;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  // Estimate point normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud.makeShared());
  ne.setKSearch (50);
  ne.compute (*cloud_normals);
  
  seg.setOptimizeCoefficients (true);
  seg.setModelType (5);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.1);
  seg.setMaxIterations (1000);
  seg.setRadiusLimits (0, 0.1);
  // seg.setInputCloud (cloud.makeShared ());//pcl::pclPointcloud2Ptrには対応していない
  seg.setInputCloud (cloud.makeShared());//pcl::pclPointcloud2Ptrには対応していない
  seg.setInputNormals (cloud_normals);
  seg.segment (inliers, coefficients);

  pcl::PointCloud<pcl::PointXYZ> inliers_cloud;

  for (size_t i = 0; i < inliers.indices.size (); ++i) { 
    inliers_cloud.push_back(cloud.points[inliers.indices[i]]);
  }
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(inliers_cloud, output);//toROSMsgを使用するとheader情報など消える？
  output.header.frame_id = "camera_depth_frame";
  pub_output.publish(output);

  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub_coefficients.publish (ros_coefficients);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "example");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_coefficients = nh.advertise<pcl_msgs::ModelCoefficients> ("model_coefficients", 1);
  pub_output = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}