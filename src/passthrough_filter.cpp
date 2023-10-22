#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>

#include <iostream>
#include <string>

ros::Publisher pub;
std::string field_name = "x";
float upper_limit = 0.0;
float lower_limit = 3.0;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  float now = ros::Time::now().toSec();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCL data type
  pcl::fromROSMsg(*cloud_msg, *cloud);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*cloud_filtered);

  sensor_msgs::PointCloud2::Ptr output_msg (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_filtered, *output_msg);

  pub.publish (output_msg);

  float duration = ros::Time::now().toSec() - now;
  ROS_INFO("Duration %.5f[sec]", duration);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "paththrough_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("field_name", field_name);
  pnh.getParam("upper_limit", upper_limit);
  pnh.getParam("lower_limit", lower_limit);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}