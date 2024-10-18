#include "catenary_checker/catenary_checker.hpp"
#include "catenary_checker/parabola.hpp"
#include <chrono>

using namespace pcl;

float getParabolaPoints(Parabola &parabola, const pcl::PointXYZ &A, const pcl::PointXYZ &B, 
                      pcl::PointCloud<pcl::PointXYZ> &p, float delta_t) {
  // Project to 2D the init and goal points
  auto plane = getVerticalPlane(A, B);
  Point2D A_(A.y * plane.a - A.x * plane.b, A.z);
  Point2D B_(B.y * plane.a - B.x * plane.b, B.z);

  auto parabola2d_points = parabola.getPoints(A_.x, B_.x, delta_t);
  pcl::PointCloud<pcl::PointXY> parabola2d;
  pcl::PointXY pcl_point;
  float length = 0.0f;
  if (parabola2d_points.size() > 0) {
    for (size_t i = 1; i < parabola2d_points.size(); i++) {
      auto p = parabola2d_points[i];
      auto q = parabola2d_points[i-1];
      pcl_point.x = p.x;
      pcl_point.y = p.y;
      parabola2d.push_back(pcl_point);
      length += sqrtf(powf(p.x - q.x, 2.0) + powf(p.y - q.y, 2.0));
    }
  }

  // Simon: This is an example to get the 3D parabola:
  p = reproject3D(parabola2d, A, B);

  return length;
}

float checkCatenary(const pcl::PointXYZ &A, const pcl::PointXYZ &B, const Scenario &scenario) {
  double ret_val = -1.0;

  // Project to 2D the init and goal points
  auto plane = getVerticalPlane(A, B);
  Point2D A_(A.y * plane.a - A.x * plane.b, A.z);
  Point2D B_(B.y * plane.a - B.x * plane.b, B.z);

  // Get the parabola
  Parabola parabola;
  if (parabola.approximateParabola(scenario, A_, B_)) {
    ret_val = parabola.getLength(A_.x, B_.x);
    
    // Simon if you want the 3D points you can use:
    //auto x = getParabolaPoints(parabola, A, B);
    
  }

  // Return the longitude of the parabola
  return ret_val;
}

float checkCatenary(const pcl::PointXYZ &A, const pcl::PointXYZ &B,const pcl::PointCloud<pcl::PointXYZ> &pc, float plane_dist, int dbscan_min_points, float dbscan_epsilon) 
{
  // Project the points to 2D
  std::shared_ptr<Scenario> scenario(PC2Obstacles(A, B, pc, plane_dist, dbscan_min_points, dbscan_epsilon));

  return checkCatenary(A, B, *scenario);
}

std::shared_ptr<Scenario> PC2Obstacles(const pcl::PointXYZ &A, const pcl::PointXYZ &B,const pcl::PointCloud<pcl::PointXYZ> &pc, float plane_dist, int dbscan_min_points, float dbscan_epsilon) {
  auto points_2d = project2D(pc, A, B, plane_dist);

  // Get the obstacles 2D clustered
  auto dbscan = clusterize(points_2d, dbscan_min_points, dbscan_epsilon);
  return getObstacles(dbscan, A, B);
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


std::shared_ptr<Scenario> getObstacles(DBSCAN *dbscan, pcl::PointXYZ A, pcl::PointXYZ B) {
  auto ret = std::make_shared<Scenario>();
  int dbscan_min_points = dbscan->getMinimumClusterSize();
  for (int i = 1; i < dbscan->getNClusters(); i++) {
    auto cluster = dbscan->getCluster(i);
    // printf("Cluster %d. Size: %lu", i, cluster.size());
    if (cluster.size() > dbscan_min_points) {
      auto curr_obstacle = toObstacle(cluster);
      ret->push_back(curr_obstacle);
    }
  }
  ret->origin.x = A.x;
  ret->origin.y = A.y;

  float dx = B.x - A.x;
  float dy = B.y - A.y;
  float dist = sqrt(dx*dx + dy*dy);
  ret->unit_vec.x = dx / dist;
  ret->unit_vec.y = dy / dist;

  ret->plane = getVerticalPlane(A,B);

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

//! @brief Decomposes a 3D scenario into a vector of planes with different directions
//! @brief passing through point A.
std::vector<Scenario> preprocessObstacle2D(const pcl::PointXYZ &A, const pcl::PointCloud<pcl::PointXYZ> &pc, int n_planes, float plane_dist, int dbscan_min_points, float dbscan_epsilon) {
  std::vector<Scenario> planes;

  // We have to sample only Pi (one plane goes to a direction and its opposite)
  float increment = M_PI / static_cast<float>(n_planes);

  float angle = 0.0;
  for (int i = 0; i < n_planes; i++, angle += increment) {
    pcl::PointXYZ B = A;
    B.x += cos(angle);
    B.y += sin(angle);

    auto scene = PC2Obstacles(A, B, pc, plane_dist, dbscan_min_points, dbscan_epsilon);
    planes.push_back(*scene);
  }

  return planes;
}



