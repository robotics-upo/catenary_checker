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


#define PRINTF_GREEN "\x1B[32m"

class CatenaryCheckerManager
{

public:
    CatenaryCheckerManager(std::string node_name_);
    // ~CatenaryCheckerManager();
    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void Init(double dist_cat_, double l_cat_max_, double ws_z_min_, double step_, bool use_analytical_method_);
    void SearchCatenary(const geometry_msgs::Point &pi_, const geometry_msgs::Point &pf_, std::vector<geometry_msgs::Point> &pts_c_);
    bool NumericalSolutionCatenary(const geometry_msgs::Point &p_reel_, const geometry_msgs::Point &p_final_, std::vector<geometry_msgs::Point> &points_catenary_);
    double getPointDistanceFullMap(bool use_dist_func_, geometry_msgs::Vector3 p_);

	bisectionCatenary bc;
    NearNeighbor nn_obs;
    Grid3d *grid_3D;
    catenaryChecker *cc;

    ros::NodeHandlePtr nh;
    ros::Subscriber point_cloud_sub_;
    sensor_msgs::PointCloud2::ConstPtr point_cloud;

    bool use_analytical_method;
    double distance_catenary_obstacle, length_tether_max, ws_z_min, step;
    double min_dist_obs_cat, length_cat_final; // to fill q_init
    double use_distance_function, catenary_state; 
};

#endif

