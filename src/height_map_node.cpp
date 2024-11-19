/**
 * @file height_map_node.cpp
 * @brief Proto version of the height_map package
 * @date 2024-11-19
 * @author ggonu
 */
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "height_map/mapping_height_map.hpp"

MappingHeightMap* mapping_height_map;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  mapping_height_map->updateHeightMap(cloud);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "height_map_node");
  ros::NodeHandle nh;

  mapping_height_map = new MappingHeightMap(0.1);
  
  ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloudCallback);

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/height_map_cloud", 1);

  ros::Rate rate(1.0);
  while (ros::ok()) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr height_map_cloud = mapping_height_map->getHeightMap();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*mapping_height_map->getHeightMap(), output);
    output.header.frame_id = "map";
    output.header.stamp = ros::Time::now();
    pub.publish(output);

    ros::spinOnce();
    rate.sleep();
  }

  delete  mapping_height_map;
  return 0;
}

