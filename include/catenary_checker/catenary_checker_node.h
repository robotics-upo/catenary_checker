#ifndef CATENARY_CHECKER_NODE_H_
#define CATENARY_CHECKER_NODE_H_

#include <vector>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <catenary_checker/catenary_checker.hpp>
#include <catenary_checker/obstacle_2d.hpp>
#include <catenary_checker/parabola.hpp>
#include <catenary_checker/grid3d.hpp>
#include "catenary_checker/near_neighbor.hpp"
#include <catenary_checker/preprocessed_scenario.hpp>

#define PRINTF_YELLOW "\x1B[33m"
#define PRINTF_GREEN "\x1B[32m"

class catenaryChecker{

public:

  catenaryChecker(ros::NodeHandlePtr nh);
  visualization_msgs::Marker pointsToMarker(const std::vector<Point> &points,
                                            const std::string frame_id, int n_lines = -1);
  void getPointCloud(const sensor_msgs::PointCloud2ConstPtr &pc_msg);
  bool analyticalCheckCatenary(const geometry_msgs::Point &pi_,
                               const geometry_msgs::Point &pf_,
                               std::vector<geometry_msgs::Point> &pts_c_);

  bool precomputedCheckCatenary(const pcl::PointXYZ &pi_,
                                const pcl::PointXYZ &pf_,
                                std::vector<geometry_msgs::Point> &pts_c_);
  double getPointDistanceFullMap(bool use_dist_func_, geometry_msgs::Point p_);
  void getDataForDistanceinformation(Grid3d *grid3D_,
                                     const sensor_msgs::PointCloud2::ConstPtr& msg,
                                     bool use_distance_function_);
  std_msgs::ColorRGBA getColor(int num);
  std_msgs::ColorRGBA getGray(int num);

  Grid3d *grid_3D;
  NearNeighbor nn_obs;

  //! Global data
  sensor_msgs::PointCloud2::ConstPtr pc;
  tf2_ros::Buffer tf_buffer;
  std::string base_frame, global_frame;
  float plane_dist = 0.5;
  // DBScan related
  int dbscan_min_points;
  float dbscan_epsilon, dbscan_gamma, dbscan_theta;
  bool use_dbscan_lines;

  bool get_catenary;

  ros::Publisher pc_publisher, marker_publisher;
  bool publish_pc = true;
  bool publish_marker = true;
  bool use_distance_function;
  double min_dist_obs_cat, length_cat_final; // to fill q_init in RRT
  double length_cat;

  float precomputing_time = -1.0;

  // Discretization of 2D planes
  std::string precomputed_file = "";
  std::unique_ptr<PreprocessedScenario> ps;

protected:

};

#endif
