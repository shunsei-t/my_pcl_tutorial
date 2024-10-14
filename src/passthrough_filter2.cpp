#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#include <pcl/filters/passthrough.h>

#include <iostream>
#include <string>

ros::Publisher pub;
std::string field_name = "x";
float limit_max = 3.0;
float limit_min = 0.0;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ros::Time start = ros::Time::now();


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*cloud_msg, *cloud);


  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(field_name);
  pass.setFilterLimits(limit_min, limit_max);
  pass.filter(*cloud_filtered);

  sensor_msgs::PointCloud2::Ptr output_msg (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_filtered, *output_msg);
  output_msg->header.frame_id = cloud_msg->header.frame_id;
  output_msg->header.stamp = cloud_msg->header.stamp;

  // Publish the data
  pub.publish (output_msg);

  ros::Duration duration = ros::Time::now() - start;
  ROS_INFO("Duration %.5ld[nsec]", duration.toNSec());
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "paththrough_filter2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("field_name", field_name);
  pnh.getParam("limit_max", limit_max);
  pnh.getParam("limit_min", limit_min);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 100000, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 100000);

  // Spin
  ros::spin ();
}