#ifndef CATENARY_CHECKER_MANAGER_H_
#define CATENARY_CHECKER_MANAGER_H_

#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <catenary_checker/catenary_checker_node.h>
#include <catenary_checker/obstacle_2d.hpp>
#include <catenary_checker/parable.hpp>
#include <catenary_checker/grid3d.hpp>
#include "catenary_checker/bisection_catenary_3D.h"
#include "catenary_checker/near_neighbor.hpp"

#define PRINTF_YELLOW "\x1B[33m"
#define PRINTF_GREEN "\x1B[32m"

class CatenaryCheckerManager
{

public:
  CatenaryCheckerManager(std::string node_name_);
  ~CatenaryCheckerManager();
  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void Init(double dist_cat_, double l_cat_max_, double ws_z_min_,
            double step_, bool use_parable_, bool use_distance_function_,
            bool use_both_ = false);
  bool SearchCatenary(const geometry_msgs::Point &pi_, const geometry_msgs::Point &pf_,
                      std::vector<geometry_msgs::Point> &pts_c_);
  //! Checks if the catenary is obstacle-free
  bool checkCatenary(const geometry_msgs::Point &A, const geometry_msgs::Point &b,
                     double l);
  //! Iteratively checks for the existence of an obstacle-free catenary
  //! between p_reel_ and p_final_ up to a maximum length
  bool NumericalSolutionCatenary(const geometry_msgs::Point &p_reel_,
                                 const geometry_msgs::Point &p_final_,
                                 std::vector<geometry_msgs::Point> &points_catenary_);
  double getPointDistanceFullMap(bool use_dist_func_, geometry_msgs::Point p_);

  inline double getPointDistanceFullMap(bool use_dist_func_, const geometry_msgs::Vector3 &v)  { 
    geometry_msgs::Point p;
    p.x = v.x; p.y = v.y; p.z = v.z;
    return getPointDistanceFullMap(use_dist_func_, p);
  }

  //! @retval true If all the points are obstacle-free
  bool checkPoints(const std::vector<geometry_msgs::Point> &points);

  //! @retval true if all the points are obstacle free in straight line
  bool checkStraightCatenary(const geometry_msgs::Point &A,
                              const geometry_msgs::Point &B,
                              std::vector<geometry_msgs::Point> &p,
                              double step = 0.05);

  inline double length(const geometry_msgs::Point &A, const geometry_msgs::Point &B) const {
    return sqrt(pow(A.x - B.x, 2.0) + pow(A.y - B.y, 2.0) + pow(A.z - B.z, 2.0));
  }


	bisectionCatenary bc;
  NearNeighbor nn_obs;
  catenaryChecker *cc = NULL;

  ros::NodeHandlePtr nh;
  ros::Subscriber point_cloud_sub_;
  sensor_msgs::PointCloud2::ConstPtr point_cloud;

  bool use_parable, use_both;
  double distance_catenary_obstacle, length_tether_max, ws_z_min, step;
  double min_dist_obs_cat, length_cat_final; // to fill q_init
  double use_distance_function, catenary_state;

  bool exportStats(const std::string &filename) const;

private:
  Grid3d *grid_3D;

  // For time execution stats
  std::vector<float> execution_times_parable, execution_times_bisection;
  std::vector<float> results_parable, results_bisection;
  float planes_precomputing_time;
};

#endif

