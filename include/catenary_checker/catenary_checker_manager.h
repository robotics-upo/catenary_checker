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
    // ~CatenaryCheckerManager();
    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void Init(Grid3d *grid_3D_, double dist_cat_, double l_cat_max_, double ws_z_min_, double step_, bool use_parable_, bool use_distance_function_);
    bool SearchCatenary(const geometry_msgs::Vector3 &pi_, const geometry_msgs::Vector3 &pf_, std::vector<geometry_msgs::Vector3> &pts_c_);
    bool NumericalSolutionCatenary(const geometry_msgs::Vector3 &p_reel_, const geometry_msgs::Vector3 &p_final_, std::vector<geometry_msgs::Vector3> &points_catenary_);
    double getPointDistanceFullMap(bool use_dist_func_, geometry_msgs::Vector3 p_);

	bisectionCatenary bc;
    NearNeighbor nn_obs;
    catenaryChecker *cc;

    ros::NodeHandlePtr nh;
    ros::Subscriber point_cloud_sub_;
    sensor_msgs::PointCloud2::ConstPtr point_cloud;

    bool use_parable;
    double distance_catenary_obstacle, length_tether_max, ws_z_min, step;
    double min_dist_obs_cat, length_cat_final; // to fill q_init
    double use_distance_function, catenary_state; 

    Grid3d *grid_3D;

private:

};

#endif

