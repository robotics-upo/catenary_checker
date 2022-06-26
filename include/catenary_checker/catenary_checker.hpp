#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <dbscan_line/dbscan_lines.h>

#ifndef __CATENARY_CHECKER_LIB__
#define __CATENARY_CHECKER_LIB__

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

pcl::PointCloud<pcl::PointXY> project2D(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointXYZ &p1, pcl::PointXYZ &p2, const float max_dist);

PlaneParams getVerticalPlane(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

pcl::PointCloud<pcl::PointXYZ> reproject3D(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in, pcl::PointXYZ &p1, pcl::PointXYZ &p2);

DBSCAN *clusterize(const pcl::PointCloud<pcl::PointXY> &pc_2d, int minPts, float epsilon);
DBSCAN *clusterize_lines(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in, int minPts, float epsilon, float gamma, float theta);

#endif
