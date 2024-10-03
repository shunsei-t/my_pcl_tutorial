#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/centroid.h>

#include <iostream>
#include <string>

ros::Publisher pub;
float mean_k = 10;
float thresh = 0.5;

void
cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  ros::Time start = ros::Time::now();

	Eigen::Vector4f xyz_centroid;
	pcl::compute3DCentroid(*cloud, xyz_centroid);

  geometry_msgs::PointStamped pointst;
  pointst.header.frame_id = cloud->header.frame_id;
  pointst.header.stamp = ros::Time::now();
  pointst.point.x = xyz_centroid[0];
  pointst.point.y = xyz_centroid[1];
  pointst.point.z = xyz_centroid[2];

  pub.publish(pointst);

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
  pub = nh.advertise<geometry_msgs::PointStamped> ("COG", 1);

  // Spin
  ros::spin ();
}