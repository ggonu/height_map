#ifndef MAPPING_HEIGHT_MAP_HPP_
#define MAPPING_HEIGHT_MAP_HPP_

#include "generate_height_map.hpp"

#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/concave_hull.h>

#include <Eigen/Dense>

#include <vector>
#include <limits>
#include <map>


class MappingHeightMap {
 public:
  MappingHeightMap(float cell_size);
  ~MappingHeightMap();

  /**
   * @brief Updates the height map with the given point cloud.
   *
   * This function takes an input point cloud and updates the height map by
   * gridifying the point cloud and then updating the height points.
   *
   * @param iCloud Input point cloud to update the height map.
   */
  void updateHeightMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iCloud);
  void resetHeightMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iCloud);

  void fillHeightMapInteriors();

  /**
   * @brief Interpolates the height map.
   * @param iCloud Input point cloud to interpolate the height map.
   */
  float interpolateHeight(float x, float y);

  bool isPointInsideHull(float x, float y) const;

  /**
   * @brief Returns the size of each cell in the height map.
   * @return cell_size_
   */
  float getCellSize() const;

  /**
   * @brief Get the generated 2.5D height map.
   * @return The output height map as a point cloud.
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getHeightMap() const;

  /**
   * @brief Calculate the properties of the plane.
   * @return A pair of centroid and orientation of the plane.
   */
  std::pair<Eigen::Vector3f, Eigen::Vector3f> calculatePlaneProperties();

  std::map<uint32_t, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmentPlanes(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iCloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateConcaveHull(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iCloud, float alpha);

  std::vector<Eigen::Vector2f> projectTo2D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iHull);

  bool isPointInsidePolygon(const Eigen::Vector2f& point, const std::vector<Eigen::Vector2f>& polygon);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr fillConcaveHull(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iCloud, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iHull, float resolution);

 private:
  HeightMap height_map_;

  float cell_size_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr height_map_cloud_;
};


#endif // MAPPING_HEIGHT_MAP_HPP_