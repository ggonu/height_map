#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <height_map/mapping_height_map.hpp>

MappingHeightMap* height_map;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (!cloud->empty()) {
        height_map->resetHeightMap(cloud);
        height_map->fillHeightMapInteriors(); // TODO: Comment this line
        auto height_map_cloud = height_map->getHeightMap();

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*height_map_cloud, output);
        output.header = msg->header;


        visualization_msgs::MarkerArray marker_array;
        int id = 0;

        for (const auto& point : height_map_cloud->points) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = msg->header.frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = "plane_marker" + std::to_string(id);
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = point.x;
            marker.pose.position.y = point.y;
            marker.pose.position.z = point.z / 2.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = height_map->getCellSize();
            marker.scale.y = height_map->getCellSize();
            marker.scale.z = point.z;

            marker.color.r = 0.6f;
            marker.color.g = 0.6f;
            marker.color.b = 0.6f;
            marker.color.a = 0.8f;

            marker_array.markers.push_back(marker);
        }

        static ros::Publisher height_map_pub = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("/height_map/recieved_cloud", 1);
        height_map_pub.publish(output);

        static ros::Publisher marker_pub = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("/height_map/plane_markers", 1);
        marker_pub.publish(marker_array);
    } else {
        ROS_WARN("Input point cloud is empty. Cannot update height map.");
    }
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

