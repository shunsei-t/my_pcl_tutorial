#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <iostream>
#include <string>

ros::Publisher pub;
float mean_k = 10;
float thresh = 0.5;

void
cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  ros::Time start = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (mean_k);
  sor.setStddevMulThresh (thresh);
  sor.filter (*cloud_filtered);

  pub.publish (*cloud_filtered);

  ros::Duration duration = ros::Time::now() - start;
  ROS_INFO("Duration %.5ld[nsec]", duration.toNSec());
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "statical_outlier_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("mean_k", mean_k);
  pnh.getParam("thresh", thresh);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}