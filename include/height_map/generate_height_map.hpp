#ifndef HEIGHT_MAP_HPP_
#define HEIGH_MAP_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <limits>
#include <map>
#include <vector>

/**
 * @brief Class for generating height map from point cloud
 */
class HeightMap {
 public:
  HeightMap(float cell_size);
  ~HeightMap();

  /**
   * @brief Grid cell structure
   * @param height_ height of the cell
   * @param points points in the cell
   */
  struct GridCell {
    float height_;
    std::vector<pcl::PointXYZ> points;

    void checkHeight(const pcl::PointXYZ& point) {
      if (point.z > height_) {
        height_ = point.z;
      }
    }
  };

  /**
   * @brief Gridify input cloud
   * @param iCloud input cloud
   * @param oCloud output cloud
   * @param cell_size size of cell
   */
  void gridifyCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& iCloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& oCloud,
    float cell_size);

  /**
   * @brief Assign height into grid
   * @param iCloud input cloud
   * @param oCloud output cloud
   */
  void assignHeightIntoGrid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& icloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& ocloud);

 private:
  float cell_size_;
};

#endif  // HEIGHT_MAP_HPP_