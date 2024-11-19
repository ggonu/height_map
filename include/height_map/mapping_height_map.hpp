#ifndef MAPPING_HEIGHT_MAP_HPP_
#define MAPPING_HEIGHT_MAP_HPP_

#include "generate_height_map.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <limits>


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
  void updateHeightMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& iCloud);

  /**
   * @brief Interpolates the height map.
   * @param iCloud Input point cloud to interpolate the height map.
   */
  void interpolation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& iCloud);

  /**
   * @brief Returns the size of each cell in the height map.
   * @return cell_size_
   */
  float getCellSize() const;

  /**
   * @brief Get the generated 2.5D height map.
   * @return The output height map as a point cloud.
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr getHeightMap() const;

 private:
  HeightMap height_map_;
  float cell_size_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr height_map_cloud_;
};



#endif // MAPPING_HEIGHT_MAP_HPP_