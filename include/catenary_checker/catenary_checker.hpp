#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <dbscan_line/dbscan_lines.h>
#include "catenary_checker/obstacle_2d.hpp"
#include "parable.hpp"

#ifndef __CATENARY_CHECKER_LIB__
#define __CATENARY_CHECKER_LIB__

//! @brief Checks for the existence of a Catenary between two 3D points
//! @param A First point
//! @param B Second point
//! @param dist Maximum distance from an obstacle to the plane to be included
//! @param obstacles Obstacles in the environment
//! @return The length of the catenary, or -1 if not found
float checkCatenary(const pcl::PointXYZ &A, const pcl::PointXYZ &B,
		    const pcl::PointCloud<pcl::PointXYZ> &pc, float dist);

class PlaneParams {
public:
  float a, b, c, d;

  inline float getSignedDistance(const pcl::PointXYZ &p) {
    return a*p.x + b*p.y + c*p.z + d;
  }

  inline std::string toString() const {
    std::ostringstream os;

    os << "Plane: " << a << "·x + " << b << "·y + " << c << "·z + " << d << "= 0";

    return os.str();
  }
};

pcl::PointCloud<pcl::PointXY> project2D(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
					const pcl::PointXYZ &p1, const pcl::PointXYZ &p2,
					const float max_dist);

PlaneParams getVerticalPlane(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

pcl::PointCloud<pcl::PointXYZ> reproject3D(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in,
					  const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

pcl::PointCloud<pcl::PointXYZ> getParablePoints(Parable &parable, const pcl::PointXYZ &A, const pcl::PointXYZ &B);

DBSCAN *clusterize(const pcl::PointCloud<pcl::PointXY> &pc_2d, int minPts, float epsilon);
DBSCAN *clusterize_lines(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in, int minPts,
			 float epsilon, float gamma, float theta);

std::vector<Obstacle2D> getObstacles(DBSCAN *dbscan);

Obstacle2D toObstacle(const std::vector<Point> &obs);

#endif
