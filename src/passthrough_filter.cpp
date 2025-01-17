#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>

#include <iostream>
#include <string>

ros::Publisher pub;
std::string field_name = "x";
float limit_max = 3.0;
float limit_min = 0.0;

void
cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  ros::Time start = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(field_name);
  pass.setFilterLimits(limit_min, limit_max);
  pass.filter(*cloud_filtered);


  pub.publish (*cloud_filtered);

  ros::Duration duration = ros::Time::now() - start;
  ROS_INFO("Duration %.5ld[nsec]", duration.toNSec());
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "paththrough_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("field_name", field_name);
  pnh.getParam("limit_max", limit_max);
  pnh.getParam("limit_min", limit_min);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/d405/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}