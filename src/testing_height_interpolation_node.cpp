#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/surface/concave_hull.h>

class HullHeightMapNode {
public:
  HullHeightMapNode(ros::NodeHandle& nh) {
    // Subscriber for input Point Cloud (containing hull points)
    hull_cloud_sub_ = nh.subscribe("/height_map/sorted_hull_cloud", 1, &HullHeightMapNode::pointCloudCallback, this);

    // Publisher for the resulting Height Map
    grid_map_pub_ = nh.advertise<grid_map_msgs::GridMap>("/height_map/grid_map", 1);

    // Node parameters
    nh.param<float>("cell_size", cell_size_, 0.01f);
    nh.param<std::string>("frame_id", frame_id_, "map");
  }

private:
  ros::Subscriber hull_cloud_sub_;
  ros::Publisher grid_map_pub_;
  float cell_size_;
  std::string frame_id_;

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert input PointCloud2 to PCL format
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Initialize GridMap
    grid_map::GridMap map({"height"});
    map.setFrameId(frame_id_);
    map.setGeometry(grid_map::Length(2.0, 2.0), cell_size_); // Set map size and resolution

    // Process each segmented plane in the point cloud
    processHullPlanes(cloud, map);

    // Publish the resulting GridMap
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map, message);
    grid_map_pub_.publish(message);

    ROS_INFO("Published Height Map.");
  }

  void processHullPlanes(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, grid_map::GridMap& map) {
    // Assuming each plane is separated by different colors in the RGB field
    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmented_planes;

    // Separate planes based on RGB values
    for (const auto& point : cloud->points) {
      uint32_t rgb = (static_cast<uint32_t>(point.r) << 16) |
                     (static_cast<uint32_t>(point.g) << 8) |
                     static_cast<uint32_t>(point.b);
      if (segmented_planes.find(rgb) == segmented_planes.end()) {
        segmented_planes[rgb] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
      }
      segmented_planes[rgb]->points.push_back(point);
    }

    // Process each plane
    for (const auto& segment : segmented_planes) {
      auto hull_cloud = segment.second;

      // Compute the plane equation (ax + by + cz + d = 0)
      Eigen::Vector4f plane_coefficients;
      if (!computePlaneFromHull(hull_cloud, plane_coefficients)) {
        ROS_WARN("Failed to compute plane for hull.");
        continue;
      }

      // Populate the GridMap with height values
      populateHeightMap(hull_cloud, plane_coefficients, map);
    }
  }

  bool computePlaneFromHull(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& hull_cloud, Eigen::Vector4f& coefficients) {
    if (hull_cloud->points.size() < 3) {
      ROS_WARN("Not enough points to compute a plane.");
      return false;
    }

    Eigen::MatrixXf points(hull_cloud->points.size(), 3);
    for (size_t i = 0; i < hull_cloud->points.size(); ++i) {
      points(i, 0) = hull_cloud->points[i].x;
      points(i, 1) = hull_cloud->points[i].y;
      points(i, 2) = hull_cloud->points[i].z;
    }

    // Perform PCA to find the plane
    Eigen::Vector3f centroid = points.colwise().mean();
    Eigen::MatrixXf centered = points.rowwise() - centroid.transpose();
    Eigen::Matrix3f covariance = (centered.adjoint() * centered) / points.rows();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);

    // Normal vector is the eigenvector corresponding to the smallest eigenvalue
    Eigen::Vector3f normal = solver.eigenvectors().col(0);
    float d = -normal.dot(centroid);

    coefficients << normal, d;
    return true;
  }

  void populateHeightMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& hull_cloud,
                         const Eigen::Vector4f& plane_coefficients,
                         grid_map::GridMap& map) {
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      map.getPosition(*it, position);

      // Check if the position is inside the convex hull
      if (isInsideConvexHull(hull_cloud, position)) {
        // Calculate the height using the plane equation
        float height = (-plane_coefficients[0] * position.x() -
                        plane_coefficients[1] * position.y() -
                        plane_coefficients[3]) /
                       plane_coefficients[2];
        float& cell_value = map.at("height", *it);

        // Update the height only if it's greater
        if (std::isnan(cell_value)) {
          cell_value = height;
        } else {
          cell_value = std::max(cell_value, height);
        }
      }
    }
  }

  bool isInsideConvexHull(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& hull_cloud, const grid_map::Position& position) {
    // Ray-casting algorithm to check if a point is inside a convex hull (2D projection)
    int crossings = 0;
    for (size_t i = 0; i < hull_cloud->points.size(); ++i) {
      const auto& p1 = hull_cloud->points[i];
      const auto& p2 = hull_cloud->points[(i + 1) % hull_cloud->points.size()];

      if (((p1.y > position.y()) != (p2.y > position.y())) &&
          (position.x() < (p2.x - p1.x) * (position.y() - p1.y) / (p2.y - p1.y) + p1.x)) {
        crossings++;
      }
    }
    return (crossings % 2) != 0; // True if inside
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "hull_height_map_node");
  ros::NodeHandle nh("~");

  HullHeightMapNode node(nh);
  ros::spin();

  return 0;
}
