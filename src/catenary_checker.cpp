#include "catenary_checker/catenary_checker.hpp"
#include "catenary_checker/parabola.hpp"
#include <chrono>

pcl::PointCloud<pcl::PointXYZ> reproject3D(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in,
					   const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

pcl::PointCloud<pcl::PointXYZ> getParabolaPoints(Parabola &parabola, const pcl::PointXYZ &A, const pcl::PointXYZ &B, float delta_t) {
  // Project to 2D the init and goal points
  auto plane = getVerticalPlane(A, B);
  Point2D A_(A.y * plane.a - A.x * plane.b, A.z);
  Point2D B_(B.y * plane.a - B.x * plane.b, B.z);

  auto parabola2d_points = parabola.getPoints(A_.x, B_.x, delta_t);
  pcl::PointCloud<pcl::PointXY> parabola2d;
  pcl::PointXY pcl_point;
  for (auto &p:parabola2d_points) {
    pcl_point.x = p.x;
    pcl_point.y = p.y;
    parabola2d.push_back(pcl_point);
  }

  // Simon: This is an example to get the 3D parabola:
  return reproject3D(parabola2d, A, B);
}

float checkCatenary(const pcl::PointXYZ &A, const pcl::PointXYZ &B,const pcl::PointCloud<pcl::PointXYZ> &pc, float plane_dist, int dbscan_min_points, float dbscan_epsilon) 
{

  double ret_val = -1.0;

  // Project the points to 2D
  auto points_2d = project2D(pc, A, B, plane_dist);

  // Get the obstacles 2D clustered
  auto dbscan = clusterize(points_2d, dbscan_min_points, dbscan_epsilon);
  auto scenario = getObstacles(dbscan);

  // Project to 2D the init and goal points
  auto plane = getVerticalPlane(A, B);
  Point2D A_(A.y * plane.a - A.x * plane.b, A.z);
  Point2D B_(B.y * plane.a - B.x * plane.b, B.z);

  // Get the parabola
  Parabola parabola;
  if (parabola.approximateParabola(scenario, A_, B_)) {
    ret_val = parabola.getLength(A_.x, B_.x);
    
    // Simon if you want the 3D points you can use:
    auto x = getParabolaPoints(parabola, A, B);
    
  }

  // Return the longitude of the parabola
  return ret_val;
}

pcl::PointCloud<pcl::PointXY> project2D(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
					const pcl::PointXYZ &p1,
					const pcl::PointXYZ &p2, const float max_dist)
{
  // std::cout << "project2D sizes: cloud_in:[" << cloud_in.size()  << "] , p1:[" << p1.x << " " << p1.y << " " << p1.z <<"] , p2:[" << p2.x << " " << p2.y << " " << p2.z <<  "] , max_dist:[" << max_dist << std::endl;

  PlaneParams plane = getVerticalPlane(p1, p2);
  pcl::PointCloud<pcl::PointXY> ret;  

  const std::chrono::steady_clock::time_point start(std::chrono::steady_clock::now());

  // Get the x' coordinate of p1 and p2

  float x_1_prima = -p1.x * plane.b + p1.y * plane.a;
  float x_2_prima = -p2.x * plane.b + p2.y * plane.a;

  // std::cout << " x_1_prima = " << x_1_prima << " , x_2_prima = " << x_2_prima << std::endl;

  float min_x = std::min(x_1_prima, x_2_prima);
  float max_x = std::max(x_1_prima, x_2_prima);
  float max_y = std::max(p1.z, p2.z);

  // std::cout << " min_x = " << min_x << " , max_x = " << max_x << " , max_y = " << max_y << std::endl;

  for (int i = cloud_in.size() - 1; i >= 0 ; i--) {
    const pcl::PointXYZ &p = cloud_in[i];
    float dist = plane.getSignedDistance(p);
        
    if (fabsf(dist) < max_dist) {
            
      // Get the point of the plane
      pcl::PointXYZ p_plane(p.x - plane.a * dist, p.y - plane.b * dist, p.z);

      // std::cout << "Adding Point. (" << p.x << ", " << p.y << ", "
      // << p.z << ")\t Abs Dist: " << fabsf(dist) << "\n";

      // Translate to 2D --> x coord is:  - p.x * plane.b + p.y * plane.a
      pcl::PointXY projected_point;
      projected_point.x = p_plane.y * plane.a - p_plane.x * plane.b;
      projected_point.y = p_plane.z;

      // Before adding the points, check if they pass these

      if (projected_point.x > min_x && projected_point.x < max_x && projected_point.y < max_y) {
	      ret.push_back(projected_point);
      }
    }
  }
  const std::chrono::steady_clock::time_point end(std::chrono::steady_clock::now());

  // std::cout << "Project2d. Cloud in size: " << cloud_in.size() << std::endl;
  // std::cout << "Got plane: " << plane.toString() << std::endl;
  const auto t = std::chrono::duration_cast<std::chrono::microseconds>( end - start ).count();
  // std::cout << "Elapsed time: " << t << " us\n";

  return ret;
}

pcl::PointCloud<pcl::PointXYZ> reproject3D(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in, const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
{
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


DBSCAN *clusterize(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in, int minPts, float epsilon)
{
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

DBSCAN *clusterize_lines(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in,
			 int minPts, float epsilon, float gamma, float theta)
{
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


std::vector<Obstacle2D> getObstacles(DBSCAN *dbscan) {
  std::vector<Obstacle2D> ret;
  int dbscan_min_points = dbscan->getMinimumClusterSize();
  for (int i = 1; i < dbscan->getNClusters(); i++) {
    auto cluster = dbscan->getCluster(i);
    // printf("Cluster %d. Size: %lu", i, cluster.size());
    if (cluster.size() > dbscan_min_points) {
      auto curr_obstacle = toObstacle(cluster);
      ret.push_back(curr_obstacle);
    }
  }

  return ret;
}

Obstacle2D toObstacle(const std::vector<Point> &obs) {
  Obstacle2D ret;

  Point2D p;
  for (auto &x:obs) {
    p.x = x.x;
    p.y = x.y;
    ret.push_back(p);
  }

  if (ret.size() > 1)
    ret.calculateConvexHull();

  return ret;
}
