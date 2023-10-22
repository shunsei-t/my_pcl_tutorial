#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <string>

ros::Publisher pub;
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> );
static pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud (new pcl::PointCloud<pcl::PointXYZ> );
static float resolution = 1.0;
static std::string frame_id;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::fromROSMsg (*input, *cloud);
  frame_id = input->header.frame_id;
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "octree_change_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("octree_resolution", resolution);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    // Octree resolution - side length of octree voxels
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

    if (last_cloud->size()){
        // Add points from cloudA to octree
        octree.setInputCloud (last_cloud);
        octree.addPointsFromInputCloud ();

        // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
        octree.switchBuffers ();

            // Add points from cloudB to octree
        octree.setInputCloud (cloud);
        octree.addPointsFromInputCloud ();

        std::vector<int> newPointIdxVector;

        // Get vector of point indices from octree voxels which did not exist in previous buffer
        octree.getPointIndicesFromNewVoxels (newPointIdxVector);

        pcl::PointCloud<pcl::PointXYZ> output_cloud;
        // Output points
        for (std::size_t i = 0; i < newPointIdxVector.size (); ++i)
            output_cloud.push_back((*cloud)[newPointIdxVector[i]]);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(output_cloud, output);//toROSMsgを使用するとheader情報など消える？
        output.header.frame_id = frame_id;

        pub.publish(output);
    }
    
    pcl::copyPointCloud(*cloud, *last_cloud);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}