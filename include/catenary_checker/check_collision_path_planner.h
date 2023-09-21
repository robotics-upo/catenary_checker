#ifndef CHECK_COLLISION_PATH_PLANNER_H_
#define CHECK_COLLISION_PATH_PLANNER_H_

#include <list>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "catenary_checker/bisection_catenary_3D.h"
#include "catenary_checker/grid3d.hpp"
#include "catenary_checker/near_neighbor.hpp"
#include "catenary_checker/get_parable_parameter.hpp"

#include <Eigen/StdVector>

#define PRINTF_REGULAR "\x1B[0m"
#define PRINTF_RED "\x1B[31m"
#define PRINTF_GREEN "\x1B[32m"

using namespace std;

class checkCollisionPathPlanner
{
	public:
        checkCollisionPathPlanner(std::string node_name_, Grid3d *grid_3D_, sensor_msgs::PointCloud2::ConstPtr pc_, geometry_msgs::Vector3 p_reel_ugv_, double d_obs_ugv_, double d_obs_uav_, double d_obs_tether_);
        bool CheckStatus(trajectory_msgs::MultiDOFJointTrajectory mt_, std::vector<double> ct_);
        bool CheckStatus(vector<geometry_msgs::Vector3> v1_, vector<geometry_msgs::Quaternion> vq1_, vector<geometry_msgs::Vector3 >v2_, vector<parable_parameters> v3_);
        double getPointDistanceFullMap(bool ugv_obstacle_, geometry_msgs::Vector3 p_, int pose_, string msg_);
        geometry_msgs::Vector3 getReelNode(const geometry_msgs::Vector3 p_, const geometry_msgs::Quaternion q_);
        double getYawFromQuaternion(double x_, double y_, double z_, double w_);

	Grid3d *grid_3D;
        NearNeighbor nn_obs_ugv;
        ros::NodeHandlePtr nh;

        sensor_msgs::PointCloud2::ConstPtr pc_obs_ugv;
        geometry_msgs::Vector3 p_reel_ugv;
        double distance_obstacle_ugv, distance_obstacle_uav, distance_tether_obstacle;
        std::string node_name;

	int count_ugv_coll, count_uav_coll, count_tether_coll;

	protected:
};


#endif