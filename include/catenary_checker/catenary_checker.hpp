#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <dbscan_line/dbscan_lines.h>
#include "catenary_checker/obstacle_2d.hpp"
#include "catenary_checker/2d_projection.hpp"
#include "parable.hpp"

#ifndef __CATENARY_CHECKER_LIB__
#define __CATENARY_CHECKER_LIB__

//! @brief Checks for the existence of a Catenary between two 3D points with 2D obstacles
//! @param A First point
//! @param B Second point
//! @param scenario The 2D scenario
float checkCatenary(const pcl::PointXYZ &A, const pcl::PointXYZ &B, const Scenario scenario);

//! @brief Checks for the existence of a Catenary between two 3D points in a PC
//! @param A First point
//! @param B Second point
//! @param pc Point cloud where the obstacles will be obtained
//! @param dist Maximum distance from an obstacle to the plane to be included
//! @param obstacles Obstacles in the environment
//! @return The length of the catenary, or -1 if not found
float checkCatenary(const pcl::PointXYZ &A, const pcl::PointXYZ &B, const pcl::PointCloud<pcl::PointXYZ> &pc, float plane_dist, int dbscan_min_points, float dbscan_epsilon);

pcl::PointCloud<pcl::PointXYZ> getParablePoints(Parable &parable, const pcl::PointXYZ &A, const pcl::PointXYZ &B, float delta_t=0.05);

std::vector<Scenario> preprocessObstacle2D(const pcl::PointXYZ &A, const pcl::PointCloud<pcl::PointXYZ> &pc, int n_planes, float plane_dist, int dbscan_min_points, float dbscan_epsilon);

// Clustering and 2D Obstacles related

Scenario PC2Obstacles(const pcl::PointXYZ &A, const pcl::PointXYZ &B,const pcl::PointCloud<pcl::PointXYZ> &pc, float plane_dist, int dbscan_min_points, float dbscan_epsilon);

DBSCAN *clusterize(const pcl::PointCloud<pcl::PointXY> &pc_2d, int minPts, float epsilon);
DBSCAN *clusterize_lines(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in, int minPts,
			 float epsilon, float gamma, float theta);

Scenario getObstacles(DBSCAN *dbscan);

Obstacle2D toObstacle(const std::vector<Point> &obs);

#endif
