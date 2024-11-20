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

void MappingHeightMap::fillHeightMapInteriors() {
  if (height_map_cloud_->empty()) {
    std::cout << "[WARNING]: Height map cloud is empty. Cannot fill interior." << std::endl;
    return;
  }

  // Find the bounding box of the height map
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*height_map_cloud_, min_pt, max_pt);

  for (float x = min_pt.x; x <= max_pt.x; x += cell_size_) {
    for (float y = min_pt.y; y <= max_pt.y; y += cell_size_) {
      if (isPointInsideHull(x, y)) {
        float height = interpolateHeight(x, y);

        pcl::PointXYZ p;
        p.x = x;
        p.y = y;
        p.z = height;
        height_map_cloud_->points.push_back(p);
      }
    }
  }
}

float MappingHeightMap::interpolateHeight(float x, float y) {
  float min_distance = std::numeric_limits<float>::max();
  float interpolated_height = 0.0f;
  for (const auto& point : height_map_cloud_->points) {
    float distance = std::sqrt(std::pow(point.x - x, 2) + std::pow(point.y - y, 2)); 
    if (distance < min_distance) {
      min_distance = distance;
      interpolated_height = point.z;
    }
  }
  return interpolated_height;
}

bool MappingHeightMap::isPointInsideHull(float x, float y) const {
  return true;
}

float MappingHeightMap::getCellSize() const {
  return cell_size_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MappingHeightMap::getHeightMap() const {
  return height_map_cloud_;
}