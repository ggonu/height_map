#include "ros/ros.h"

#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "height_map/mapping_height_map.hpp"

// #define PERIOD

// Create a global instance of MappingHeightMap with a cell size of 1.0
MappingHeightMap mapping_height_map(0.03);  // Adjust the cell size as needed

/* Old one
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert input PointCloud2 message to PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);

  // Update the height map using the input point cloud
  mapping_height_map.updateHeightMap(cloud);

  // Create a MarkerArray message to visualize the height map
  visualization_msgs::MarkerArray marker_array;
  int id = 0;

  // Get the updated height map point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr height_map_cloud = mapping_height_map.getHeightMap();

  // Iterate over the points in the height map cloud to create markers
  for (const auto& point : height_map_cloud->points) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_depth_optical_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "height_map";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the position and size of the cube marker
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z / 2.0;  // Center the cube vertically
    marker.pose.orientation.w = 1.0;
    marker.scale.x = mapping_height_map.getCellSize();
    marker.scale.y = mapping_height_map.getCellSize();
    marker.scale.z = point.z;

    // Set the color of the cube marker
    marker.color.r = 0.6f;
    marker.color.g = 0.6f;  // Green color for visualization
    marker.color.b = 0.6f;
    marker.color.a = 0.8f;

    marker_array.markers.push_back(marker);
}

  // Publish the MarkerArray
  static ros::NodeHandle nh;
  static ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/height_map_markers", 1);
  marker_pub.publish(marker_array);
}
*/

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  mapping_height_map.updateHeightMap(cloud);
  mapping_height_map.fillHeightMapInteriors();

  visualization_msgs::MarkerArray marker_array;
  int id = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr height_map_cloud = mapping_height_map.getHeightMap();

  for (const auto& point : height_map_cloud->points) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = msg->header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "height_map";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z / 2.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = mapping_height_map.getCellSize();
    marker.scale.y = mapping_height_map.getCellSize();
    marker.scale.z = point.z;

    marker.color.r = 0.6f;
    marker.color.g = 0.6f;
    marker.color.b = 0.6f;
    marker.color.a = 0.8f;

    marker_array.markers.push_back(marker);
  }

  static ros::NodeHandle nh;
  static ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/height_map_markers", 1);
  marker_pub.publish(marker_array);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "height_map_node");
  ros::NodeHandle nh;

  // Subscribe to the point cloud topic
  ros::Subscriber point_cloud_sub = nh.subscribe("/d435i_cloud", 1, pointCloudCallback); // /plane_seg/hull_cloud

  #ifdef PERIOD
    ros::Rate rate(1.0);
    while (ros::ok()) {
      ros::spinOnce();
      rate.sleep();
    }
  #else
    ros::spin();
  #endif

  return 0;
}
