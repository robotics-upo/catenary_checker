#pragma once

#include <catenary_checker/point_2d.hpp>
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

class Scenario:public std::vector<Obstacle2D> {
public:
  PlaneParams plane; // Plane in which the obstacle was projected to
  Point2D origin, unit_vec;

  //! @brief Translates the pointcloud to a pointcloud
  inline sensor_msgs::PointCloud2 toPC(const std::string &frame_id, int seq = 0, float intensity = 1.0f) const;

  inline visualization_msgs::MarkerArray toMarkerArray(const std::string &frame_id, int seq = 0) const;

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

  inline bool loadScenario(const std::string &filename);
};

inline sensor_msgs::PointCloud2 Scenario::toPC(const std::string &frame_id,
                                               int seq, float intensity) const {
 sensor_msgs::PointCloud2 pcl_msg;
    
 //Modifier to describe what the fields are.
 sensor_msgs::PointCloud2Modifier modifier(pcl_msg);

 modifier.setPointCloud2Fields(4,
                               "x", 1, sensor_msgs::PointField::FLOAT32,
                               "y", 1, sensor_msgs::PointField::FLOAT32,
                               "z", 1, sensor_msgs::PointField::FLOAT32,
                               "intensity", 1, sensor_msgs::PointField::FLOAT32);

 //Msg header
 pcl_msg.header = std_msgs::Header();
 pcl_msg.header.stamp = ros::Time::now();
 pcl_msg.header.frame_id = frame_id;
 pcl_msg.header.seq = seq;

 pcl_msg.height = 1;
 pcl_msg.width = getTotalPoints();
 pcl_msg.is_dense = false;

 //Total number of bytes per point
 pcl_msg.point_step = 16;
 pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
 pcl_msg.data.resize(pcl_msg.row_step);


 //Iterators for PointCloud msg
 sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
 sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
 sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
 sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_msg, "intensity");

 for (const Obstacle2D &o:*this) {
   for (const Point2D &p:o) {
     pcl::PointXYZ p3 = to3D(p);
     *iterX = p3.x;
     *iterY = p3.y;
     *iterZ = p3.z;
     *iterIntensity = intensity;

     ++iterX; ++iterY; ++iterZ; ++iterIntensity;
   }
 }

 return pcl_msg;
}

inline visualization_msgs::MarkerArray Scenario::toMarkerArray(const std::string &frame_id,
                                               int seq) const {
 visualization_msgs::MarkerArray msg;
 visualization_msgs::Marker marker;
    
 //Modifier to describe what the fields are.
 marker.header = std_msgs::Header();
 marker.header.stamp = ros::Time::now();
 marker.header.frame_id = frame_id;
 marker.header.seq = seq;

 marker.id = seq;
 marker.ns = "point_cloud";
 marker.action = visualization_msgs::Marker::ADD;

 // First the point in the origin
 marker.type = visualization_msgs::Marker::SPHERE;
 marker.scale.x = marker.scale.y = marker.scale.z = 4.0;
 marker.pose.orientation.z = 0;
 marker.pose.position.x = origin.x;
 marker.pose.position.y = origin.y;
 marker.color.a = 1.0f;
 marker.color.b = 1.0f;
 msg.markers.push_back(marker);

 // Then the arrow
 marker.type = visualization_msgs::Marker::ARROW;
 marker.scale.x = marker.scale.y = marker.scale.z = 4.0;
 marker.pose.orientation.w = 1.0;
 marker.pose.position.x = origin.x;
 marker.pose.position.y = origin.y;
 geometry_msgs::Point g_p;
 marker.pose.position.x = 0.0;
 marker.pose.position.y = 0.0;

 g_p.x = origin.x; g_p.y = origin.y;
 marker.points.push_back(g_p);
 g_p.x = unit_vec.x; g_p.y += unit_vec.y;
 marker.points.push_back(g_p);
 msg.markers.push_back(marker);

 // Add the sphere list
 marker.type = visualization_msgs::Marker::SPHERE_LIST;
 marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
 marker.points.resize(getTotalPoints());
 int i = 0;
 for (const Obstacle2D &o:*this) {
   for (const Point2D &p:o) {
     pcl::PointXYZ p3 = to3D(p);
     g_p.x = p3.x;
     g_p.y = p3.y;
     g_p.z = p3.z;

     marker.points[i++] = g_p;
   }
 }
 msg.markers.push_back(marker);

 return msg;
}

// Get scenario from file
inline bool Scenario::loadScenario(const std::string &filename) {
  bool ret_val = true;
  clear();
  try {
    std::ifstream ifs(filename.c_str());

    YAML::Node f = YAML::Load(ifs);
    origin = Point2D(f["origin"]);
    unit_vec = Point2D(f["unit_vec"]);
    plane = PlaneParams(f["plane"]);

    printf("Load Scenario: Origin = %s. \t Unit vec: %s\n", origin.toString().c_str(),
             unit_vec.toString().c_str());

    for (const auto &x:f["obstacles"]) {
      Obstacle2D o(x);
      this->push_back(o);
    }
  } catch (std::exception &e) {
    std::cerr << "Could not load scenario. e: " << e.what() << std::endl;
    ret_val = false;
  }

  return ret_val;
}



inline YAML::Emitter &operator << (YAML::Emitter &out, const Scenario &s) {
  out << YAML::BeginMap;
  out << YAML::Key << "origin" << YAML::Value << s.origin;
  out << YAML::Key << "unit_vec" << YAML::Value << s.unit_vec;
  out << YAML::Key << "plane" << YAML::Value << s.plane;
  out << YAML::Key << "obstacles" << YAML::Value;
  out << YAML::BeginSeq;
  for (const auto &o:s) {
    out << o;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  return out;
}

