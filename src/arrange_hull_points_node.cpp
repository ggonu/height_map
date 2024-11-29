#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <height_map/arrange_hull_points.hpp>

#define TIMING
#ifdef TIMING
  #include <chrono>
#endif


class PointSorterNode {
public:
  PointSorterNode(ros::NodeHandle& nh) {
    sub_hull_cloud_ = nh.subscribe("/plane_seg/hull_cloud", 1, &PointSorterNode::pointCloudCallback, this);
    pub_sorted_hul_ = nh.advertise<sensor_msgs::PointCloud2>("/height_map/sorted_hull_cloud", 1);

    arranger_.setArrangeDirection(true);
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!msg) {
      ROS_WARN("Received NULL point cloud.");
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr iCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    try {
      pcl::fromROSMsg(*msg, *iCloud);
    } catch (const std::exception& e) {
      ROS_ERROR("Failed to convert ROS message to PCL point cloud: %s", e.what());
      return;
    }

    if (iCloud->points.empty()) {
      ROS_WARN("Received empty point cloud.");
      return;
    }

    #ifdef TIMING
      auto tic = std::chrono::high_resolution_clock::now();
    #endif

    std::vector<ahp::ArrangeHullPoints::HullP> sHull;
    try {
      sHull = arranger_.arrange(iCloud);
    } catch (const std::exception& e) {
      ROS_ERROR("Error during hull arrangement: %s", e.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    sCloud->reserve(sHull.size());
    for (const auto& hPoint : sHull) {
      sCloud->points.push_back(hPoint.mPoint);
      ROS_INFO(" - Sorted Point %d: (%f, %f, %f)", hPoint.mIndex, hPoint.mPoint.x, hPoint.mPoint.y, hPoint.mPoint.z);
    }

    #ifdef TIMING
      auto toc = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = toc - tic;
      ROS_INFO("Time to arrange hull points: %f", elapsed.count());
    #endif

    sensor_msgs::PointCloud2 sMsg;
    try {
      pcl::toROSMsg(*sCloud, sMsg);
    } catch (const std::exception& e) {
      ROS_ERROR("Failed to convert PCL point cloud to ROS message: %s", e.what());
      return;
    }

    sMsg.header = msg->header;
    pub_sorted_hul_.publish(sMsg);
    ROS_INFO("Published sorted hull cloud.");
  }

private:
  ros::Subscriber sub_hull_cloud_;
  ros::Publisher pub_sorted_hul_;
  ahp::ArrangeHullPoints arranger_;

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "arrange_hull_points_node");
  ros::NodeHandle nh;

  PointSorterNode node(nh);

  ros::spin();

  return 0;
}
