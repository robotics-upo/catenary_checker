#pragma once

#include <catenary_checker/point_2d.hpp>
#include <catenary_checker/point_3d.hpp>
#include <catenary_checker/obstacle_2d.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <vector>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <fstream>
#include <visualization_msgs/MarkerArray.h>
#include "2d_projection.hpp"
#include "catenary_checker/grid3d.hpp"

std_msgs::ColorRGBA getColor(int num);

class Scenario:public std::vector<Obstacle2D> {
public:
  PlaneParams plane; // Plane in which the obstacle was projected to
  Point2D unit_vec;

  //! @brief Default constructor
  Scenario () {}


  //! @brief Key constructor: makes a projection from EDF in a discrete way between A and B (ignores the rest of the workspace)  
  Scenario(const Grid3d &grid, const Point3D &A, const Point3D &B, float min_dist = 0.2f, float res = 0.05f); 

  //! @brief Translates the pointcloud to a pointcloud
  sensor_msgs::PointCloud2 toPC(const std::string &frame_id, int seq = 0, float intensity = 1.0f) const;

  visualization_msgs::MarkerArray toMarkerArray(const std::string &frame_id, int seq = 0, bool different_color = false) const;

  inline Point2D to2D(const pcl::PointXYZ &p) const {
    return plane.project2D(p);
  }
  
  inline pcl::PointXYZ to3D(const Point2D &p) const {
    return plane.project3D(p);
  }

  inline size_t getTotalPoints() const {
    size_t ret = 0;
    for (auto o:*this) {
      ret += o.size();
    }
    return ret;
  }

  bool loadScenario(const std::string &filename);

  void simplify();

  std::string toString() const;
};

YAML::Emitter &operator << (YAML::Emitter &out, const Scenario &s);