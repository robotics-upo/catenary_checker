#include "catenary_checker/catenary_checker.hpp"
#include <chrono>

pcl::PointCloud<pcl::PointXY> project2D(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointXYZ &p1, 
                                        pcl::PointXYZ &p2, const float max_dist) {
    PlaneParams plane = getVerticalPlane(p1, p2);
    pcl::PointCloud<pcl::PointXY> ret;

    const std::chrono::steady_clock::time_point start(std::chrono::steady_clock::now());

    for (int i = cloud_in.size() - 1; i >= 0; i--) {
        const pcl::PointXYZ &p = cloud_in[i];
        float dist = plane.getSignedDistance(p);
        
        if (fabsf(dist) < max_dist) {
            
            // Get the point of the plane
            pcl::PointXYZ p_plane(p.x - plane.a * dist, 
                                  p.y - plane.b * dist, 
                                  p.z);

            // std::cout << "Adding Point. (" << p.x << ", " << p.y << ", "
	    // << p.z << ")\t Abs Dist: " << fabsf(dist) << "\n";

            // Translate to 2D --> x coord is:  - p.x * plane.b + p.y * plane.a
            pcl::PointXY projected_point;
            projected_point.x = p_plane.y * plane.a - p_plane.x * plane.b;
            projected_point.y = p_plane.z;

            ret.push_back(projected_point);
        }
    }
    const std::chrono::steady_clock::time_point end(std::chrono::steady_clock::now());

    std::cout << "Project2d. Cloud in size: " << cloud_in.size() << std::endl;
    std::cout << "Got plane: " << plane.toString() << std::endl;
    const auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>( end - start ).count();
    std::cout << "Elapsed time: " << duration_us << " us\n";

    return ret;
}

pcl::PointCloud<pcl::PointXYZ> reproject3D(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in, pcl::PointXYZ &p1, pcl::PointXYZ &p2) {
    pcl::PointCloud<pcl::PointXYZ> ret;
    pcl::PointXYZ delta(p2);
    delta.x -= p1.x;
    delta.y -= p1.y;
    delta.z = 0;
    float dist = sqrtf(delta.x * delta.x + delta.y * delta.y); // Normalize
    delta.x /= dist;
    delta.y /= dist;

    auto plane = getVerticalPlane(p1, p2);
    
    for (int i = cloud_2d_in.size() - 1; i >= 0; i--) {
        const pcl::PointXY &p_2d = cloud_2d_in[i];
        
        // Get the point of the plane and translate back to 3D
        pcl::PointXYZ p_3d(p_2d.x * delta.x - plane.a * plane.d, 
                           p_2d.x * delta.y - plane.b * plane.d, 
                           p_2d.y);

        ret.push_back(p_3d);
    }

    return ret;
}

PlaneParams getVerticalPlane(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) {
    PlaneParams plane;

    // The normal vector will be (v2 - v1).crossproduct(0,0,1) -->
    // (x2 - x1, y2 - y1, z2 - z1) x (0, 0, 1) -->  n = (y2 - y1, x1 - x2, 0) 
    plane.a = p2.y - p1.y;
    plane.b = p1.x - p2.x;
    plane.c = 0;

    // Normalize:
    float dist = sqrtf(plane.a * plane.a + plane.b * plane.b);
    plane.a /= dist;
    plane.b /= dist;
    
    // Get d by substituting another point ( n * p1 + d = 0 ) --> d = - n * p1
    plane.d = - plane.a * p1.x - plane.b * p1.y;

    return plane;
}


DBSCAN *clusterize(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in, int minPts, float epsilon) {
    // Convert pointcloud to DBSCan format
    std::vector<Point> points;
    points.reserve(cloud_2d_in.size());

    Point db_p;
    for (auto &p:cloud_2d_in) {
        db_p.x = p.x;
        db_p.y = p.y;
        db_p.z = 0.0;

        points.push_back(db_p);
    }
    DBSCAN *dbscan = new DBSCAN(minPts, epsilon, points);

    dbscan->run();

    return dbscan;
}

DBSCAN *clusterize_lines(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in, int minPts, float epsilon, float gamma, float theta) {
    // Convert pointcloud to DBSCan format
    std::vector<Point> points;
    points.reserve(cloud_2d_in.size());

    Point db_p;
    for (auto &p:cloud_2d_in) {
        db_p.x = p.x;
        db_p.y = p.y;
        db_p.z = 0.0;

        points.push_back(db_p);
    }
    DBSCAN *dbscan = new DBSCANLines(minPts, epsilon, points, gamma, theta);

    dbscan->run();

    return dbscan;
}
