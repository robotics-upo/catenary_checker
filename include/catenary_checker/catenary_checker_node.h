#ifndef CATENARY_CHECKER_NODE_H_
#define CATENARY_CHECKER_NODE_H_

#include <vector>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <catenary_checker/catenary_checker.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <catenary_checker/obstacle_2d.hpp>
#include <catenary_checker/parable.hpp>

class catenaryChecker{

public:

  catenaryChecker(ros::NodeHandlePtr nh);
  visualization_msgs::Marker pointsToMarker(const std::vector<Point> &points, const std::string frame_id, int n_lines = -1);
  void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr &pc_msg);
  bool analyticalCheckCatenary(const geometry_msgs::Point &pi_, const geometry_msgs::Point &pf_);
  std_msgs::ColorRGBA getColor(int num);
  std_msgs::ColorRGBA getGray(int num);

  //! Global data
  sensor_msgs::PointCloud2 pc;
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

protected:

};

#endif
