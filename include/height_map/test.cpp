/**
 * @file test.cpp
 * @brief Test file for height_map package
 * @date 2024-11-18
 * @author ggonu
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "generate_height_map.hpp"


int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "height_map_test_node");
  ros::NodeHandle nh;

  // Create ROS publishers
  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/height_map_test_output", 1);
  ros::Publisher pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/height_map_test_markers", 1);

  // Create a sample point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Add some points to the point cloud
  cloud->points.push_back(pcl::PointXYZ(1.0, 1.0, 2.0));
  cloud->points.push_back(pcl::PointXYZ(1.5, 1.0, 3.0));
  cloud->points.push_back(pcl::PointXYZ(2.0, 2.0, 1.0));
  cloud->points.push_back(pcl::PointXYZ(3.0, 3.0, 4.0));
  cloud->points.push_back(pcl::PointXYZ(3.1, 3.0, 5.0));

  cloud->points.push_back(pcl::PointXYZ(1.3, 1.1, 1.8));
  cloud->points.push_back(pcl::PointXYZ(1.6, 1.8, 3.0));
  cloud->points.push_back(pcl::PointXYZ(2.7, 2.6, 1.2));
  cloud->points.push_back(pcl::PointXYZ(3.7, 3.2, 4.5));
  cloud->points.push_back(pcl::PointXYZ(3.1, 3.0, 5.6));

  // Set the grid cell size
  float cell_size = 0.1;

  // Create an instance of HeightMap
  HeightMap height_map(cell_size);

  // Perform grid-based analysis of the point cloud and generate output cloud
  height_map.gridifyCloud(cloud, output_cloud, cell_size);

  // Convert the output point cloud to ROS message
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*output_cloud, output);
  output.header.frame_id = "map";

  // Create MarkerArray for visualization
  visualization_msgs::MarkerArray marker_array;
  int marker_id = 0;
  for (const auto& point : output_cloud->points) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "height_map_test_cells";
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z / 2.0; // Place the cube from the ground
    marker.scale.x = cell_size;
    marker.scale.y = cell_size;
    marker.scale.z = point.z; // Height of the cube
    marker.color.a = 0.8;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker_array.markers.push_back(marker);
  }

  // Publish the point cloud and markers periodically
  ros::Rate rate(1.0);
  while (ros::ok()) {
    output.header.stamp = ros::Time::now();
    pub_cloud.publish(output);
    pub_markers.publish(marker_array);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

