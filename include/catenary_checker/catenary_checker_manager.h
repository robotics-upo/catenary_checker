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
    // ~CatenaryCheckerManager();
    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void PointCloudObstaclesCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void Init(Grid3d *grid_3D_, double d_obs_tether_, double d_obs_ugv_, double d_obs_uav_, double l_cat_max_, double ws_z_min_, 
								double step_, bool use_parabola_, bool use_distance_function_, geometry_msgs::Vector3 p_reel_ugv_, bool j_l_o_s_, bool use_catenary_as_tether);
    bool SearchCatenary(const geometry_msgs::Vector3 &pi_, const geometry_msgs::Vector3 &pf_, std::vector<geometry_msgs::Vector3> &pts_c_);
    bool NumericalSolutionCatenary(const geometry_msgs::Vector3 &p_reel_, const geometry_msgs::Vector3 &p_final_, std::vector<geometry_msgs::Vector3> &points_catenary_);
    double getPointDistanceObstaclesMap(bool use_dist_func_, geometry_msgs::Vector3 p_);
    double getPointDistanceObstaclesMap(bool use_dist_func_, geometry_msgs::Vector3 p_, int pose_, string msg_);
    bool CheckStatusCollision(trajectory_msgs::MultiDOFJointTrajectory mt_, std::vector<double> ct_);
    // bool CheckStatusCollision(vector<geometry_msgs::Vector3> v1_, vector<geometry_msgs::Quaternion> vq1_, vector<geometry_msgs::Vector3 >v2_, vector<tether_parameters> v3_);
    bool CheckStatusTetherCollision(vector<geometry_msgs::Vector3> v1_, vector<geometry_msgs::Quaternion> vq1_, vector<geometry_msgs::Vector3 >v2_, vector<tether_parameters> v3_, vector<float> length_);
    bool CheckFreeCollisionPoint(geometry_msgs::Vector3 p_, string mode_, int pose_);
    geometry_msgs::Vector3 getReelNode(const geometry_msgs::Vector3 p_, const geometry_msgs::Quaternion q_);
    double getYawFromQuaternion(double x_, double y_, double z_, double w_);
    bool computeStraight(const geometry_msgs::Vector3 &p_reel_, const geometry_msgs::Vector3 &p_final_, std::vector<geometry_msgs::Vector3> &points_catenary_);

	bisectionCatenary bc;
    NearNeighbor nn_obs;
    catenaryChecker *cc;

    ros::NodeHandlePtr nh;
    ros::Subscriber point_cloud_sub_, point_cloud_ugv_obs_sub_;
    sensor_msgs::PointCloud2::ConstPtr point_cloud;
    geometry_msgs::Vector3 p_reel_ugv;

    bool use_parabola;
    bool just_line_of_sigth, use_catenary_as_tether; // This variable allow the class just compute the straigth state of the tether 
    double length_tether_max, ws_z_min, step;
    double distance_obstacle_ugv, distance_obstacle_uav, distance_tether_obstacle;
    double min_dist_obs_cat, length_cat_final; // to fill q_init
    double use_distance_function, catenary_state; 
    double param_cat_x0, param_cat_y0, param_cat_a ;
	int count_ugv_coll, count_uav_coll, count_tether_coll;


    Grid3d *grid_3D;
    vector<int> v_pos_coll_tether;


private:

};

#endif

