#include "generate_height_map.hpp"

#define TIMING
#ifdef TIMING
#include <chrono>
#endif

#define DEBUG

HeightMap::HeightMap(float cell_size) : cell_size_(cell_size) {}

HeightMap::~HeightMap() {}

void HeightMap::gridifyCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& iCloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& oCloud,
  float cell_size) {
  // TODO: Implement this function
  std::map<std::pair<int, int>, GridCell> grid_cell;
  for (const auto& point : iCloud->points) {
    int x_idx = static_cast<int>(std::floor(point.x / cell_size));
    int y_idx = static_cast<int>(std::floor(point.y / cell_size));

    #ifdef TIMING
      auto tic = std::chrono::high_resolution_clock::now();
    #endif
    grid_cell[{x_idx, y_idx}].checkHeight(point);
    #ifdef TIMING
      auto toc = std::chrono::high_resolution_clock::now();
      std::cout << "[DEBUG]: Gridify took " << 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() << "ms" << std::endl;
    #endif
  }

  for (const auto& cell : grid_cell) {
    int x_idx = cell.first.first;
    int y_idx = cell.first.second;
    const GridCell& grid_cell = cell.second;


    pcl::PointXYZ p;
    p.x = x_idx * cell_size;
    p.y = y_idx * cell_size;
    p.z = grid_cell.height_;

    oCloud->points.push_back(p);
  }


  #ifdef DEBUG
    for (const auto& cell : grid_cell) {
      int x_idx = cell.first.first;
      int y_idx = cell.first.second;
      const GridCell& grid_cell = cell.second;

      std::cout << "Cell (" << x_idx << ", " << y_idx << ")" << std::endl;
      std::cout << "Height: " << grid_cell.height_ << std::endl;
  }
  #endif
}

void HeightMap::assignHeightIntoGrid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& iCloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& oCloud) {
  // TODO: Check gridifyCloud function (line 27: assign Height into the grid)
}
