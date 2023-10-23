#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
static float x = 0.1, y = 0.1, z = 0.1;

void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
{
  ros::Time start = ros::Time::now();
  // PoincCloud2トピックをpcl::PCLPointCloud2Constとして受け取ることで暗示的に変換している
  pcl::PCLPointCloud2 cloud_filtered;

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (x, y, z);
  sor.filter (cloud_filtered);

  // Publish the data
  pub.publish (cloud_filtered);

  ros::Duration duration = ros::Time::now() - start;
  ROS_INFO("Duration %.5ld[nsec]", duration.toNSec());
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "voxel_grid_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("leaf_x", x);
  pnh.getParam("leaf_y", y);
  pnh.getParam("leaf_z", z);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}