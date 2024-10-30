#include "catenary_checker/2d_projection.hpp"
#include <chrono>

pcl::PointCloud<pcl::PointXY> project2D(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
					const pcl::PointXYZ &p1,
					const pcl::PointXYZ &p2, const float max_dist, const float min_z)
{
  PlaneParams plane = getVerticalPlane(p1, p2);
  pcl::PointCloud<pcl::PointXY> ret;  

  float min_x = std::min(p1.x, p2.x);
  float max_x = std::max(p1.x, p2.x);
  float min_y = std::min(p1.y, p2.y);
  float max_y = std::max(p1.y, p2.y);
  
  float max_z = std::max(p1.z, p2.z);

  const std::chrono::steady_clock::time_point start(std::chrono::steady_clock::now());
  auto s = cloud_in.size();
  ret.reserve(cloud_in.size());


  // Get the data from the plane
  float a = plane.a;
  float b = plane.b;
  // float c = plane.a; C is always zero (vertical plane)
   float d = plane.a;
  
  #pragma omp parallel for num_threads(16) shared(ret) 
  for (size_t i = 0; i < s ; i++) {
    // Before adding the points, check if they are in the proper range
    if (p1.x < min_x || p1.x > max_x || p1.y < min_y || p1.y > max_y || p1.z < min_z || p1.z > max_z)
      continue; 

    const pcl::PointXYZ &p = cloud_in[i];
    float dist = a * p.x + b * p.y + d;
        
    if (dist < max_dist && dist > -max_dist) {
      // Translate to 2D --> x coord is:  - p.x * plane.b + p.y * plane.a
      pcl::PointXY projected_point;
      projected_point.x = p.y * plane.a - p.x * plane.b;
      projected_point.y = p.z;

      // #pragma omp atomic update        TODO: not working --> get the size externally and put it in the proper position
        ret.push_back(projected_point);
    }
  }
  #pragma omp barrier
  const std::chrono::steady_clock::time_point end(std::chrono::steady_clock::now());

  std::cout << "Project2d. Cloud in size: " << cloud_in.size() << std::endl;
  std::cout << "Got plane: " << plane.toString() << std::endl;
  std::cout << "Points projected: " << ret.size() << std::endl;
  const auto t = std::chrono::duration_cast<std::chrono::microseconds>( end - start ).count();
  std::cout << "Elapsed time: " << t << " us\n";

  return ret;
}

std::vector<pcl::PointCloud<pcl::PointXY> > project2D_theta(const float theta, const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
					const pcl::PointXYZ &min,
					const pcl::PointXYZ &max, PlaneParams &p, const float dist_planes, const float min_z)
{
  p = getVerticalPlane(theta);
  std::vector<pcl::PointCloud<pcl::PointXY> > ret;  


  // Get the corners with less and max d
  float min_d = 1e20; float max_d = -1e20;

  std::vector <pcl::PointXYZ> corners;
  corners.push_back(min);
  corners.push_back(max);
  pcl::PointXYZ aux;
  aux.x = min.x; aux.y = max.y;
  corners.push_back(aux);
  aux.x = max.x; aux.y = min.y;
  corners.push_back(aux);

  int min_corner = -1;
  int max_corner = -1;
  
  for (size_t i = 0; i < corners.size(); i++) {
    float d = p.getSignedDistance(corners[i]);
    if (d < min_d) {
      min_corner = i;
      min_d = d;
    }
    if (d > max_d) {
      max_corner = i;
      max_d = d;
    }
  }
  int length = max_d - min_d;

  // Get number of planes
  float inv_dist_planes = 1.0 / dist_planes;
  int n_planes = std::ceil(length *inv_dist_planes);
  ret.resize(n_planes);
  
  size_t s = cloud_in.size();
  for (auto &x : ret) {
    x.reserve(s);
  }

  p.d = -min_d;

  std::cout << "Number of planes in direction " <<theta << " is : " << n_planes << std::endl;

  // #pragma omp parallel for num_threads(16)
  for (size_t i = 0; i < s ; i++) {
    const pcl::PointXYZ &current_point = cloud_in[i];
    // Before adding the points, check if they are in the proper range
    if (current_point.x < min.x || current_point.x > max.x || 
        current_point.y < min.y || current_point.y > max.y ||
        current_point.z < min.z || current_point.z > max.z || current_point.z < min_z )
      continue; 

    float dist = p.getSignedDistance(current_point);
    int n_plane = abs((int)(dist * inv_dist_planes));

    pcl::PointXY projected_point;
    projected_point.x = current_point.y * p.a - current_point.x * p.b;
    projected_point.y = current_point.z;
    // printf("Trying to put a point in plane: %d\n", n_plane);
    ret[n_plane].points.push_back(projected_point);
  }
  // #pragma omp barrier
  
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
    
  // Get d by substituting a point ( n * p1 + d = 0 ) --> d = - n * p1
  plane.d = - plane.a * p1.x - plane.b * p1.y;

  return plane;
}


