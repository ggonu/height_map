#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <grid_map_msgs/GridMap.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// #include <opencv2/core.hpp>


class HeightMapNode {
public:
    HeightMapNode(ros::NodeHandle& nh) {
        point_sub_ = nh.subscribe("/d435i_cloud", 1, &HeightMapNode::pointCallback, this);
        height_map_pub_ = nh.advertise<grid_map_msgs::GridMap>("/height_map", 1);
    }

private:
    void pointCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        sensor_msgs::PointCloud2 height_map;
        pcl::toROSMsg(cloud, height_map);
        height_map.header.frame_id = "camera_link";
        height_map_pub_.publish(height_map);
    }

    ros::Subscriber point_sub_;
    ros::Publisher height_map_pub_;
};
