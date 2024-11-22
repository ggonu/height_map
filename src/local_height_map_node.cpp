#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <height_map/mapping_height_map.hpp>


/**
 * Calculate and visualize the height map
 * Calculates the normal of the plane
 * Calculates the centroid of the plane
 * Visualizes the height map as a point cloud
 * Visualizes the plane as a marker
 */


MappingHeightMap* height_map;

void publishMarkers(const ros::Publisher& marker_pub,
                    const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& segmented_planes) {
  int id = 0;
  for (const auto& segment : segmented_planes) {
    uint32_t rgb = segment.first;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = segment.second;

    height_map->resetHeightMap(cloud);
    auto [centroid, normal] = height_map->calculatePlaneProperties();

    visualization_msgs::Marker marker;
    marker.header.frame_id = cloud->header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "plane_marker" + std::to_string(id);
    marker.id = id++;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = centroid.x();
    marker.pose.position.y = centroid.y();
    marker.pose.position.z = centroid.z();

    tf2::Vector3 normal_vector(normal.x(), normal.y(), normal.z());
    tf2::Vector3 x_axis(1.0, 0.0, 0.0);
    tf2::Vector3 rotation_axis = x_axis.cross(normal_vector);
    double angle = x_axis.angle(normal_vector);
    tf2::Quaternion quaternion;
    quaternion.setRotation(rotation_axis, angle);

    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w();

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.01;

    marker.color.r = ((rgb >> 16) & 0xFF) / 255.0;
    marker.color.g = ((rgb >> 8) & 0xFF) / 255.0;
    marker.color.b = (rgb & 0xFF) / 255.0;
    marker.color.a = 0.8f;

    marker_pub.publish(marker);
  }
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    auto segmented_planes = height_map->segmentPlanes(cloud);

    static ros::Publisher marker_pub = ros::NodeHandle().advertise<visualization_msgs::Marker>("/height_map/plane_markers", 1);

    publishMarkers(marker_pub, segmented_planes);

    ROS_INFO("Published markers for %lu planes", segmented_planes.size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_height_map_node");
    ros::NodeHandle nh("~");

    float cell_size;
    nh.param<float>("cell_size", cell_size, 0.01f);

    height_map = new MappingHeightMap(cell_size);

    ros::Subscriber sub = nh.subscribe("/plane_seg/hull_cloud", 1, pointCloudCallback);
    ros::spin();

    delete height_map;
    return 0;
}

