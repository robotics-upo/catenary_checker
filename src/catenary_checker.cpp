#include "catenary_checker/catenary_checker.hpp"
#include "catenary_checker/parable.hpp"
#include <chrono>

pcl::PointCloud<pcl::PointXYZ> getParablePoints(Parable &parable, const pcl::PointXYZ &A, const pcl::PointXYZ &B, float delta_t) {
  // Project to 2D the init and goal points
  auto plane = getVerticalPlane(A, B);
  Point2D A_(A.y * plane.a - A.x * plane.b, A.z);
  Point2D B_(B.y * plane.a - B.x * plane.b, B.z);

  auto parable2d_points = parable.getPoints(A_.x, B_.x, delta_t);
  pcl::PointCloud<pcl::PointXY> parable2d;
  pcl::PointXY pcl_point;
  for (auto &p:parable2d_points) {
    pcl_point.x = p.x;
    pcl_point.y = p.y;
    parable2d.push_back(pcl_point);
  }

  // Simon: This is an example to get the 3D parable:
  return reproject3D(parable2d, A, B);
}

float checkCatenary(const pcl::PointXYZ &A, const pcl::PointXYZ &B, const Scenario scenario) {
  double ret_val = -1.0;

  // Project to 2D the init and goal points
  auto plane = getVerticalPlane(A, B);
  Point2D A_(A.y * plane.a - A.x * plane.b, A.z);
  Point2D B_(B.y * plane.a - B.x * plane.b, B.z);

  // Get the parable
  Parable parable;
  if (parable.approximateParable(scenario, A_, B_)) {
    ret_val = parable.getLength(A_.x, B_.x);
    // Simon if you want the 3D points you can use:
    auto x = getParablePoints(parable, A, B);
  }

  return ret_val;
}

float checkCatenary(const pcl::PointXYZ &A, const pcl::PointXYZ &B,const pcl::PointCloud<pcl::PointXYZ> &pc, float plane_dist, int dbscan_min_points, float dbscan_epsilon) 
{
  // Project the points to 2D
  auto scenario = PC2Obstacles(A, B, pc, plane_dist, dbscan_min_points, dbscan_epsilon);

  return checkCatenary(A, B, scenario);
}

Scenario PC2Obstacles(const pcl::PointXYZ &A, const pcl::PointXYZ &B,const pcl::PointCloud<pcl::PointXYZ> &pc, float plane_dist, int dbscan_min_points, float dbscan_epsilon) {
  auto points_2d = project2D(pc, A, B, plane_dist);

  // Get the obstacles 2D clustered
  auto dbscan = clusterize(points_2d, dbscan_min_points, dbscan_epsilon);
  return getObstacles(dbscan);
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
    planes.push_back(scene);
  }

  return planes;
}
