#include "mapping_height_map.hpp"


MappingHeightMap::MappingHeightMap(float cell_size)
  : height_map_(cell_size), cell_size_(cell_size), height_map_cloud_(new pcl::PointCloud<pcl::PointXYZ>) {
}

MappingHeightMap::~MappingHeightMap() {
}

void MappingHeightMap::updateHeightMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& iCloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Generate height map from the input point cloud
  height_map_.gridifyCloud(iCloud, temp_cloud, cell_size_);

  for (const auto& point : iCloud->points) {
    bool isMapUpdated = false;
    for (auto& existing_point : height_map_cloud_->points) {
      if (std::fabs(existing_point.x - point.x) < 1e-2 &&
          std::fabs(existing_point.y - point.y) < 1e-2) {
        if (existing_point.z < point.z) {
          existing_point.z = point.z;
        }
        isMapUpdated = true;
        break;
      }
    }
    if (!isMapUpdated) {
      height_map_cloud_->points.push_back(point);
    }
  }
}

float MappingHeightMap::getCellSize() const {
  return cell_size_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MappingHeightMap::getHeightMap() const {
  return height_map_cloud_;
}