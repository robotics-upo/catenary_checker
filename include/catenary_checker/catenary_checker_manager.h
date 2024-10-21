#ifndef CATENARY_CHECKER_MANAGER_H_
#define CATENARY_CHECKER_MANAGER_H_

#include <vector>
#include "ceres/ceres.h"

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <catenary_checker/catenary_checker_node.h>
#include <catenary_checker/obstacle_2d.hpp>
#include <catenary_checker/parabola.hpp>
#include "catenary_checker/bisection_catenary_3D.h"
#include "catenary_checker/near_neighbor.hpp"
#include "catenary_checker/grid3d.hpp"
#include "catenary_checker/get_tether_parameter.hpp"


#define PRINTF_YELLOW "\x1B[33m"
#define PRINTF_GREEN "\x1B[32m"

class CatenaryCheckerManager
{

public:
  CatenaryCheckerManager(std::string node_name_);
  ~CatenaryCheckerManager();
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void pointCloudObstaclesCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void init(Grid3d *grid_3D_, double d_obs_tether_, double d_obs_ugv_, double d_obs_uav_,
            double l_cat_max_, double ws_z_min_,
            double step_, bool use_parabola_, bool use_distance_function_,
            geometry_msgs::Point p_reel_ugv_, bool j_l_o_s_, bool use_catenary_as_tether, 
            bool use_both_ = false);

  bool searchCatenary(const geometry_msgs::Point &pi_, const geometry_msgs::Point &pf_,
                      std::vector<geometry_msgs::Point> &pts_c_);
  //! Checks if the catenary is obstacle-free
  bool checkCatenary(const geometry_msgs::Point &A, const geometry_msgs::Point &b,
                     double l);
  //! Iteratively checks for the existence of an obstacle-free catenary
  //! between p_reel_ and p_final_ up to a maximum length
  bool numericalSolutionCatenary(const geometry_msgs::Point &p_reel_,
                                 const geometry_msgs::Point &p_final_,
                                 std::vector<geometry_msgs::Point> &points_catenary_);
  double getPointDistanceFullMap(bool use_dist_func_, geometry_msgs::Point p_);

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
  
  double getPointDistanceObstaclesMap(bool use_dist_func_, geometry_msgs::Point p_);
  double getPointDistanceObstaclesMap(bool use_dist_func_, geometry_msgs::Point p_, int pose_, string msg_);
  bool checkStatusCollision(trajectory_msgs::MultiDOFJointTrajectory mt_, std::vector<float> ct_);
  bool checkStatusTetherCollision(vector<geometry_msgs::Point> v1_, vector<geometry_msgs::Quaternion> vq1_, vector<geometry_msgs::Point >v2_, vector<tether_parameters> v3_, vector<float> length_, bool use_catenary_as_tether_);
  bool checkFreeCollisionPoint(geometry_msgs::Point p_, string mode_, int pose_);
  geometry_msgs::Point getReelNode(const geometry_msgs::Point p_, const geometry_msgs::Quaternion q_);
  double getYawFromQuaternion(double x_, double y_, double z_, double w_);
  bool computeStraight(const geometry_msgs::Point &p_reel_, const geometry_msgs::Point &p_final_, std::vector<geometry_msgs::Point> &points_catenary_);
  bool checkFreeCollisionTether(geometry_msgs::Point p1_, geometry_msgs::Point p2_, tether_parameters p_, float l_, int pos_);

	bisectionCatenary bc;
  NearNeighbor nn_obs;
  catenaryChecker *cc = NULL;

  ros::NodeHandlePtr nh;
  ros::Subscriber point_cloud_sub_, point_cloud_ugv_obs_sub_;
  sensor_msgs::PointCloud2::ConstPtr point_cloud;

  double distance_obstacle_ugv, distance_obstacle_uav, distance_tether_obstacle;
  double length_tether_max, ws_z_min, step;
  double min_dist_obs_cat, length_cat_final; // to fill q_init
  double use_distance_function, catenary_state;
  bool just_line_of_sight, use_catenary_as_tether; // This variable allow the class just compute the straigth state of the tether 
  bool use_parabola, use_both;
  bool exportStats(const std::string &filename) const;

  vector<int> v_pos_coll_tether;
  Grid3d *grid_3D;

  geometry_msgs::Point p_reel_ugv;

  // Catenary parameters
  double param_cat_x0, param_cat_y0, param_cat_a;
  // Collision statuses
	int count_ugv_coll, count_uav_coll, count_tether_coll, count_total_tether_coll_;
  
private:

  // For time execution stats
  std::vector<float> execution_times_parabola, execution_times_bisection;
  std::vector<float> results_parabola, results_bisection;
  std::vector<std::pair<geometry_msgs::Point, geometry_msgs::Point> > problems;
  float planes_precomputing_time;
};

#endif

