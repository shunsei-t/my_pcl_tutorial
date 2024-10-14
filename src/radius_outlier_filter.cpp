#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/radius_outlier_removal.h>

#include <iostream>
#include <string>

ros::Publisher pub;
float radius = 0.5;
float neighbors = 2;

void
cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  ros::Time start = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // build the filter
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(radius);
  outrem.setMinNeighborsInRadius (neighbors);
  outrem.setKeepOrganized(true);
  // apply filter
  outrem.filter (*cloud_filtered);

  pub.publish (*cloud_filtered);

  ros::Duration duration = ros::Time::now() - start;
  ROS_INFO("Duration %.5f[sec]", duration.toSec());
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "radius_outlier_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("radius", radius);
  pnh.getParam("neighbors", neighbors);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}