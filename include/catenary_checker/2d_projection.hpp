#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <dbscan_line/dbscan_lines.h>
#include "catenary_checker/obstacle_2d.hpp"
#include "parable.hpp"

#ifndef __2D_PROYECTION_LIB__
#define __2D_PROYECTION_LIB__

typedef std::vector<Obstacle2D> Scenario;

// ----------- Plane projection related classes / functions -----------

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

// 3D Reprojection

pcl::PointCloud<pcl::PointXYZ> reproject3D(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in,
                                           const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

#endif
