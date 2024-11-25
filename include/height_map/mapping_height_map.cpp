#include "mapping_height_map.hpp"

#define DEBUG

MappingHeightMap::MappingHeightMap(float cell_size)
  : height_map_(cell_size), cell_size_(cell_size), height_map_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>) {
}

MappingHeightMap::~MappingHeightMap() {
}

void MappingHeightMap::updateHeightMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iCloud) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

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

void MappingHeightMap::resetHeightMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iCloud) {
  height_map_cloud_->clear();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  height_map_.gridifyCloud(iCloud, temp_cloud, cell_size_);

  height_map_cloud_ = temp_cloud;
}

void MappingHeightMap::fillHeightMapInteriors() {
  #ifdef DEBUG
    if (height_map_cloud_->empty()) {
      std::cout << "[WARNING]: Height map cloud is empty. Cannot fill interior." << std::endl;
      return;
    }
  #endif

  // Find the bounding box of the height map
  pcl::PointXYZRGB min_pt, max_pt;
  pcl::getMinMax3D(*height_map_cloud_, min_pt, max_pt);

  for (float x = min_pt.x; x <= max_pt.x; x += cell_size_) {
    for (float y = min_pt.y; y <= max_pt.y; y += cell_size_) {
      if (isPointInsideHull(x, y)) {
        float height = interpolateHeight(x, y);

        pcl::PointXYZRGB p;
        p.x = x;
        p.y = y;
        p.z = height;
        height_map_cloud_->points.push_back(p);
      }
    }
  }
}

float MappingHeightMap::interpolateHeight(float x, float y) {
  auto [centroid, orientation] = calculatePlaneProperties();

  #ifdef DEBUG
    std::cout << "Centroid: " << centroid.transpose() << std::endl;
    std::cout << "Orientation: " << orientation.transpose() << std::endl;
  #endif

  // Find the closest point to (x, y)
  float min_distance = std::numeric_limits<float>::max();
  float interpolated_height = 0.0f;

  for (const auto& point : height_map_cloud_->points) {
    float distance = std::sqrt(std::pow(point.x - x, 2) + std::pow(point.y - y, 2));
    if (distance < min_distance) {
      min_distance = distance;
      interpolated_height = point.z;
    }
  }

  #ifdef DEBUG
    if (min_distance > cell_size_) {
      std::cout << "[WARNING]: Interpolated height is not accurate. Distance: " << min_distance << " < " << cell_size_ << std::endl;
    }
  #endif

  return interpolated_height;
}

bool MappingHeightMap::isPointInsideHull(float x, float y) const {
  return true;
}

float MappingHeightMap::getCellSize() const {
  return cell_size_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MappingHeightMap::getHeightMap() const {
  return height_map_cloud_;
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> MappingHeightMap::calculatePlaneProperties() {
  if (height_map_cloud_->empty()) {
    ROS_WARN("Height map cloud is empty. Cannot calculate plane properties.");
    return {Eigen::Vector3f(0.0, 0.0, 0.0), Eigen::Vector3f(0.0, 0.0, 1.0)};
  }
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*height_map_cloud_, centroid);

  // pcl::PCA<pcl::PointXYZRGB> pca;
  // pca.setInputCloud(height_map_cloud_); // height_map_cloud_

  // Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
  // Eigen::Vector3f plane_orientation = eigen_vectors.col(0);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

  ne.setInputCloud(height_map_cloud_);
  // ne.setSearchRadius(0.03);
  ne.setKSearch(10);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  ne.setSearchMethod(tree);
  ne.compute(*normals);
  Eigen::Vector3f avg_normal(0.0, 0.0, 0.0);
  for (const auto& normal : normals->points) {
    Eigen::Vector3f n(normal.normal_x, normal.normal_y, normal.normal_z);
    avg_normal += n;
  }
  avg_normal.normalize();

  return  {centroid.head<3>(), avg_normal};
}

std::map<uint32_t, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> MappingHeightMap::segmentPlanes(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iCloud) {
  std::map<uint32_t, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmented_planes;

  for (const auto& point : iCloud->points) {
    uint32_t rgb = (static_cast<uint32_t>(point.r) << 16) |
                   (static_cast<uint32_t>(point.g) << 8) |
                    static_cast<uint32_t>(point.b);

    if (segmented_planes.find(rgb) == segmented_planes.end()) {
      segmented_planes[rgb] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    }

  segmented_planes[rgb]->points.push_back(point);
  }

  return segmented_planes;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MappingHeightMap::generateConcaveHull(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iCloud, float alpha) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::ConcaveHull<pcl::PointXYZRGB> chull;
  chull.setInputCloud(iCloud);
  chull.setAlpha(alpha);
  chull.reconstruct(*hull);

  return hull;
}

std::vector<Eigen::Vector2f> MappingHeightMap::projectTo2D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iHull) {
  std::vector<Eigen::Vector2f> points;

  if (!iHull || iHull->points.empty()) {
    ROS_WARN("Input hull is empty. Cannot project to 2D.");
    return points;
  }

  for (const auto& point : iHull->points) {
    points.emplace_back(Eigen::Vector2f(point.x, point.y));
  }

  return points;
}

bool MappingHeightMap::isPointInsidePolygon(const Eigen::Vector2f& point, const std::vector<Eigen::Vector2f>& polygon) {
  if (polygon.empty()) {
    ROS_WARN("Polygon is empty. Cannot check if point is inside.");
    return false;
  }

  int crossings = 0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    Eigen::Vector2f p1 = polygon[i];
    Eigen::Vector2f p2 = polygon[(i + 1) % polygon.size()];

    if (((p1.y() > point.y()) != (p2.y() > point.y())) &&
        (point.x() < (p2.x() - p1.x()) * (point.y() - p1.y()) / (p2.y() - p1.y()) + p1.x())) {
      crossings++;
    }
  }

  return (crossings % 2) != 0;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MappingHeightMap::fillConcaveHull(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iCloud,
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iHull,
  float resolution) {

  if (!iHull || iHull->points.empty()) {
    ROS_WARN("Input hull is empty. Cannot fill concave hull.");
    return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filled_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<Eigen::Vector2f> hull = projectTo2D(iHull);

  if (hull.empty()) {
    ROS_WARN("Hull is empty. Cannot fill concave hull.");
    return filled_hull;
  }

  Eigen::Vector2f min_pt = hull[0];
  Eigen::Vector2f max_pt = hull[0];

  for (const auto& point : hull) {
    min_pt.x() = std::min(min_pt.x(), point.x());
    min_pt.y() = std::min(min_pt.y(), point.y());
    max_pt.x() = std::max(max_pt.x(), point.x());
    max_pt.y() = std::max(max_pt.y(), point.y());
  }

  for (float x = min_pt.x(); x <= max_pt.x(); x += resolution) {
    for (float y = min_pt.y(); y <= max_pt.y(); y += resolution) {
      Eigen::Vector2f p(x, y);
      if (isPointInsidePolygon(p, hull)) {
        pcl::PointXYZRGB nPoint;
        nPoint.x = x;
        nPoint.y = y;
        nPoint.z = 0.0;
        filled_hull->points.push_back(nPoint);
      }
    }
  }

  filled_hull->width = filled_hull->points.size();
  filled_hull->height = 1;
  filled_hull->is_dense = true;

  return filled_hull;
}