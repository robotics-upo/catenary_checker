#include "catenary_checker/catenary_checker.hpp"

pcl::PointCloud<pcl::PointXY> project2D(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointXYZ &p1, 
                                        pcl::PointXYZ &p2, const float max_dist) {
    PlaneParams plane = getVerticalPlane(p1, p2);
    pcl::PointCloud<pcl::PointXY> ret;

    for (int i = cloud_in.size() - 1; i >= 0; i--) {
        const pcl::PointXYZ &p = cloud_in[i];
        float dist = plane.getSignedDistance(p);
        if (fabsf(dist) < max_dist) {
            // Get the point of the plane
            pcl::PointXYZ p_plane(p.x - plane.a * dist, 
                                  p.y - plane.b * dist, 
                                  p.z);

            // Translate to 2D --> x coord is:  - p.x * plane.b + p.y * plane.a
            pcl::PointXY projected_point;
            projected_point.x = p_plane.y * plane.a - p_plane.x * plane.b;
            projected_point.y = p_plane.z;

            ret.push_back(projected_point);
        }
    }

    return ret;
}

pcl::PointCloud<pcl::PointXYZ> reproject_3D(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in, pcl::PointXYZ &p1, pcl::PointXYZ &p2) {
    pcl::PointCloud<pcl::PointXYZ> ret;
    pcl::PointXYZ delta(p2);
    delta.x -= p1.x;
    delta.y -= p1.y;
    delta.z = 0;
    float dist = delta.x * delta.x + delta.y * delta.y; // Normalize
    delta.x /= dist;
    delta.y /= dist;
    for (int i = cloud_2d_in.size() - 1; i >= 0; i--) {
        const pcl::PointXY &p_2d = cloud_2d_in[i];
        
        // Get the point of the plane and translate back to 3D
        pcl::PointXYZ p_3d(p_2d.x * delta.x, 
                           p_2d.x * delta.y, 
                           p_2d.y);

        ret.push_back(p_3d);
    }

    return ret;
}

PlaneParams getVerticalPlane(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) {
    PlaneParams plane;

    // The normal vector will be (v2 - v1).crossproduct(0,0,1) --> (x2-x1, y2 - y1, z2-z1) x (0, 0, 1)
    // n = (y2-y1, x1-x2, 0) --> a = y2-y1 ; b = x1-x2, c = 0
    plane.a = p2.y - p1.y;
    plane.b = p1.x - p2.x;
    plane.c = 0;

    // Normalize:
    float dist = plane.a*plane.a + plane.b*plane.b;
    plane.a /= dist;
    plane.b /= dist;
    
    // Get the d by substituting another point ( v * p1 + d = 0 ) --> d = - v*p1
    plane.d = - plane.a * p1.x - plane.b * p1.y;

    return plane;
}