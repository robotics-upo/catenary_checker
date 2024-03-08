#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <dbscan_line/dbscan_lines.h>
#include "catenary_checker/obstacle_2d.hpp"
#include "parable.hpp"

#ifndef __2D_PROYECTION_LIB__
#define __2D_PROYECTION_LIB__


// ----------- Plane projection related classes / functions -----------

class PlaneParams {
public:
  float a, b, c, d;

  inline float getSignedDistance(const pcl::PointXYZ &p) const {
    return a*p.x + b*p.y + c*p.z + d;
  }

  inline std::string toString() const {
    std::ostringstream os;

    os << "Plane: " << a << "·x + " << b << "·y + " << c << "·z + " << d << "= 0";

    return os.str();
  }

  inline pcl::PointXYZ project3D(const Point2D &p) const {
    pcl::PointXYZ p_3d(-p.x * b - a * d,
                       p.x * a - b * d,
                       p.y);
    return p_3d;
  }

  inline Point2D project2D(const pcl::PointXYZ &p) const {
    float dist = getSignedDistance(p);
    pcl:: PointXYZ p_plane(p.x - a*dist, p.y - b*dist, p.z);
    return Point2D(p_plane.y * a - p_plane.x * b, p_plane.z);
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
