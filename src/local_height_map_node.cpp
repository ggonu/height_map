#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#define DEBUG

void pointCloudCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
  grid_map::GridMap map({"height"});
  map.setFrameId(msg->markers[0].header.frame_id);

  double resolution = 0.01;
  map.setGeometry(grid_map::Length(1.0, 1.0), resolution);

  #ifdef DEBUG
    ROS_INFO("Grid map created with size %f x %f m (%i x %i cells).",
            map.getLength().x(), map.getLength().y(),
            map.getSize()(0), map.getSize()(1));
  #endif

  for (const auto& marker : msg->markers) {
    if (marker.type != visualization_msgs::Marker::CUBE) {
      ROS_WARN("Ignoring marker with type %i", marker.type);
      continue;
    }

    double x = marker.pose.position.x;
    double y = marker.pose.position.y;
    double z = marker.pose.position.z;
    double scale_x = marker.scale.x;
    double scale_y = marker.scale.y;

    tf2::Quaternion q(marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w);
    tf2::Matrix3x3 rotation(q);

    #ifdef DEBUG
      ROS_INFO("Processing marker at (%f, %f, %f) with scale %f x %f", x, y, static_cast<float>(z), scale_x, scale_y);
    #endif

    for (double dx = -scale_x / 2; dx <= scale_x / 2; dx += resolution) {
      for (double dy = -scale_y / 2; dy <= scale_y / 2; dy += resolution) {
        tf2::Vector3 local_point(dx, dy, 0.0);
        tf2::Vector3 global_point = rotation * local_point;
        global_point.setZ(-global_point.z());
        double world_x = x + global_point.x();
        double world_y = y + global_point.y();
        double world_z = z + global_point.z();

        grid_map::Position position(x + dx, y + dy);
        if (map.isInside(position)) {
          // map.atPosition("height", position) = z;
          float& height = map.atPosition("height", position);
          if (std::isnan(height)) {
            height = world_z;
          } else {
            height = std::max(height, static_cast<float>(world_z));
          }
        }
      }
    }
  }

  static ros::Publisher grid_map_pub = ros::NodeHandle().advertise<grid_map_msgs::GridMap>("/height_map/grid_map", 1);
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map, message);
  grid_map_pub.publish(message);

  ROS_INFO("Published grid map.");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_height_map_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/height_map/plane_markers", 1, pointCloudCallback);

  ros::spin();

  return 0;
}