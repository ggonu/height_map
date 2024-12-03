#ifndef ARRANGE_HULL_POINTS_HPP_
#define ARRANGE_HULL_POINTS_HPP_

#include <cmath>
#include <vector>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace ahp { // arrange hull points

class ArrangeHullPoints {
public:
  struct HullP {
    pcl::PointXYZRGB mPoint;
    int mIndex;
  };

public:
  ArrangeHullPoints() : mCounterClockwise(true) {}
  virtual ~ArrangeHullPoints() {}

  void setArrangeDirection(const bool iCounterClockwise = true) { mCounterClockwise = iCounterClockwise; }

  float computeDistancePointXY(pcl::PointXYZRGB& iPoint) const {
    return std::sqrt(std::pow(iPoint.x, 2) + std::pow(iPoint.y, 2));
  }

  int findCorePointXY(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iCloud) const {
    int coreIndex = 0;
    float maxDistance = 0.0f;

    for (size_t i = 0; i < iCloud->points.size(); ++i) {
      float distance = computeDistancePointXY(iCloud->points[i]);
      if (distance > maxDistance) {
        maxDistance = distance;
        coreIndex = i;
      }
    }
    return coreIndex;
  }

  float computePolarAngle(const pcl::PointXYZRGB& ref, const pcl::PointXYZRGB& target) const {
    float dx = target.x - ref.x;
    float dy = target.y - ref.y;
    return std::atan2(dy, dx);
    // TODO: Check if the angle is in the range of [0, 2*PI]
    // TODO: Check if the angles are same, then compare the distance
  }

  std::vector<HullP> arrange(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& iHull) const {
    if (!iHull || iHull->points.empty()) {
      throw std::runtime_error("Invalid or empty input point cloud.");
    }

    std::vector<HullP> hull;
    hull.reserve(iHull->points.size());

    for (size_t i = 0; i < iHull->points.size(); ++i) {
      HullP hp;
      hp.mPoint = iHull->points[i];
      hp.mIndex = i;
      hull.push_back(hp);
    }

    int coreIndex = findCorePointXY(iHull);
    pcl::PointXYZRGB corePoint = iHull->points[coreIndex];

    std::sort(hull.begin(), hull.end(), [this, &corePoint](const HullP& a, const HullP& b) {
      float angleA = computePolarAngle(corePoint, a.mPoint);
      float angleB = computePolarAngle(corePoint, b.mPoint);
      return mCounterClockwise ? (angleA > angleB) : (angleA < angleB);
    });

    return hull;
  }

private:
  bool mCounterClockwise;

};

} // namespace ahp


#endif // ARRANGE_HULL_POINTS_HPP_