#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>

#include <iostream>

ros::Publisher pub;
float range_min = 0.0;
float range_max = 30.0;

void
cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  ros::Time start = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_filtered->header = cloud->header;

  for (const auto point : *cloud){
    float x = point.x;
    float y = point.y;
    if((hypot(x, y) > range_min) && (hypot(x, y) < range_max)){
      cloud_filtered->push_back(point);
    }
  }


  pub.publish (*cloud_filtered);

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

  pnh.getParam("range_min", range_min);
  pnh.getParam("range_max", range_max);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}