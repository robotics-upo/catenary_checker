#include <catenary_checker/scenario.hpp>

Scenario::Scenario(const Grid3d &grid3d, const Point3D &A, const Point3D &B, float min_dist, float res) {
  float height, width;
  
  // Get the vertical plane
  Point3D delta = B - A;
  Point3D direction = delta;
  direction.normalize(res);
  
  plane = getVerticalPlane(A.toPCL(), B.toPCL());

  Point2D init, end, curr;
  init = plane.project2D(A.toPCL());
  end = plane.project2D(B.toPCL());

  init.y = 0.0;
  end.y = 6.0;

  if (init.x > end.x) {
    float aux = init.x;
    init.x = end.x;
    end.x = aux;
  }

  int i = 0, j = 0;
  int max_j = ceil((end.x - init.x) / res);
  int max_i = ceil((end.y) / res);
  vector<vector <int> > grid(max_i, std::vector<int>(max_j, 0));
  int n_obstacles = 0;
  for (i = 0, curr = init; i < max_i; curr.y += res, i++ ) {
    for (j = 0, curr.x = init.x; j < max_j; curr.x += res, j++) {
      pcl::PointXYZ curr_3d = plane.project3D(curr);

      if (grid3d.isIntoMap(curr_3d.x, curr_3d.y, curr_3d.z)) {
        // ROS_INFO("Generating Scenario. Current Point: %f, %f, %f. Distance: %f", curr_3d.x, curr_3d.y, curr_3d.z, grid3d.getPointDist(curr_3d.x, curr_3d.y, curr_3d.z));

        if (grid3d.getPointDist(curr_3d.x, curr_3d.y, curr_3d.z) < min_dist) {
          // First we have to see if there is a neighbor obstacle previously detected
          if (i > 0 && grid[i - 1][j] != 0) {
            // ROS_INFO("Updating obstacle (row). ID: %d", grid[i - 1][j]);
            
            grid[i][j] = grid[i - 1][j];
            at(grid[i][j] - 1).push_back(curr);
          } else if (j > 0 && grid[i][j - 1] != 0) {
            // ROS_INFO("Updating obstacle (column). ID: %d", grid[i][j - 1]);
            grid[i][j] = grid[i][j - 1];
            at(grid[i][j] - 1).push_back(curr);
          } else if (i > 0 && j > 0 && grid[i - 1][j - 1] != 0) {
            // ROS_INFO("Updating obstacle (column). ID: %d", grid[i - 1][j - 1]);
            grid[i][j] = grid[i - 1][j - 1];
            at(grid[i][j] - 1).push_back(curr);
          } else if (j < max_j - 1 &&  i > 0 && grid[i - 1][j + 1] != 0) {
            // ROS_INFO("Updating obstacle (column). ID: %d", grid[i][j + 1]);
            grid[i][j] = grid[i - 1][j + 1];
            at(grid[i][j] - 1).push_back(curr);
          } else {
            // New obstacles
            // ROS_INFO("New obstacle. N_obstacles: %d", n_obstacles);
            Obstacle2D o;
            o.push_back(curr);
            push_back(o);
            grid[i][j] = ++n_obstacles;
          }
        }
      }
    }
  }
}

sensor_msgs::PointCloud2 Scenario::toPC(const std::string &frame_id,
                                               int seq, float intensity) const {
 sensor_msgs::PointCloud2 pcl_msg;
    
 //Modifier to describe what the fields are.
 sensor_msgs::PointCloud2Modifier modifier(pcl_msg);

 modifier.setPointCloud2Fields(4,
                               "x", 1, sensor_msgs::PointField::FLOAT32,
                               "y", 1, sensor_msgs::PointField::FLOAT32,
                               "z", 1, sensor_msgs::PointField::FLOAT32,
                               "intensity", 1, sensor_msgs::PointField::FLOAT32);

 //Msg header
 pcl_msg.header = std_msgs::Header();
 pcl_msg.header.stamp = ros::Time::now();
 pcl_msg.header.frame_id = frame_id;
 pcl_msg.header.seq = seq;

 pcl_msg.height = 1;
 pcl_msg.width = getTotalPoints();
 pcl_msg.is_dense = false;

 //Total number of bytes per point
 pcl_msg.point_step = 16;
 pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
 pcl_msg.data.resize(pcl_msg.row_step);


 //Iterators for PointCloud msg
 sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
 sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
 sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
 sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_msg, "intensity");

 for (const Obstacle2D &o:*this) {
   for (const Point2D &p:o) {
     pcl::PointXYZ p3 = to3D(p);
     *iterX = p3.x;
     *iterY = p3.y;
     *iterZ = p3.z;
     *iterIntensity = intensity;

     ++iterX; ++iterY; ++iterZ; ++iterIntensity;
   }
 }

 return pcl_msg;
}

visualization_msgs::MarkerArray Scenario::toMarkerArray(const std::string &frame_id,
                                               int seq, bool different_color) const {
 visualization_msgs::MarkerArray msg;
 visualization_msgs::Marker marker;
    
 //Modifier to describe what the fields are.
 marker.header = std_msgs::Header();
 marker.header.stamp = ros::Time::now();
 marker.header.frame_id = frame_id;
 marker.header.seq = seq;

 marker.id = seq;
 marker.ns = "point_cloud";
 marker.action = visualization_msgs::Marker::ADD;
 marker.color = getColor(seq);
 
 // Then the arrow
 marker.type = visualization_msgs::Marker::ARROW;
 marker.scale.x = marker.scale.y = marker.scale.z = 4.0;
 marker.pose.orientation.w = 1.0;
 marker.pose.position.x = 0.0;
 marker.pose.position.y = 0.0;
 geometry_msgs::Point g_p;
 marker.pose.position.x = 0.0;
 marker.pose.position.y = 0.0;

 g_p.x = 0.0; g_p.y = 0.0;
 marker.points.push_back(g_p);
 g_p.x = unit_vec.x; g_p.y += unit_vec.y;
 marker.points.push_back(g_p);
 msg.markers.push_back(marker);

 // Add the sphere list
 marker.type = visualization_msgs::Marker::SPHERE_LIST;
 marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
 
 int n_obs = 0;
 for (const Obstacle2D &o:*this) {
   int i = 0;
   if (different_color) {
     marker.id = n_obs;
     marker.color = getColor(n_obs++);
   }
   marker.points.resize(o.size());
   for (const Point2D &p:o) {
     pcl::PointXYZ p3 = to3D(p);

     g_p.x = p3.x;
     g_p.y = p3.y;
     g_p.z = p3.z;

     marker.points[i++] = g_p;
   }
   msg.markers.push_back(marker);
 }
 

 return msg;
}

// Get scenario from file
bool Scenario::loadScenario(const std::string &filename) {
  bool ret_val = true;
  clear();
  try {
    std::ifstream ifs(filename.c_str());

    YAML::Node f = YAML::Load(ifs);
    unit_vec = Point2D(f["unit_vec"]);
    plane = PlaneParams(f["plane"]);

    for (const auto &x:f["obstacles"]) {
      Obstacle2D o(x);
      this->push_back(o);
    }
  } catch (std::exception &e) {
    std::cerr << "Could not load scenario. e: " << e.what() << std::endl;
    ret_val = false;
  }

  return ret_val;
}

std::string Scenario::toString() const {
    std::string s;

    for (const auto x:*this) {
      s.append(x.toString());
    }

    return s;
  }

YAML::Emitter &operator << (YAML::Emitter &out, const Scenario &s) {
  out << YAML::BeginMap;
  out << YAML::Key << "unit_vec" << YAML::Value << s.unit_vec;
  out << YAML::Key << "plane" << YAML::Value << s.plane;
  out << YAML::Key << "obstacles" << YAML::Value;
  out << YAML::BeginSeq;
  for (const auto &o:s) {
    out << o;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  return out;
}


void Scenario::simplify() {
  for (auto &x:*this) {
    x.simplify();
  } 
}

std_msgs::ColorRGBA getColor(int num) {
  
  // Different colors for planes
  const int n_colors = 10;
  int i = num % n_colors;
  std_msgs::ColorRGBA color;

  color.a = 1.0;
  switch (i) {
  case 0:
    color.b = 1.0;
    break;

  case 1:
    color.g = 1.0;
    break;

  case 2:
    color.r = 1.0;
    break;

  case 3:
    color.r = 1.0;
    color.b = 1.0;
    break;

  case 4:
    color.g = 1.0;
    color.b = 1.0;
    break;

  case 5:
    color.g = 1.0;
    color.r = 1.0;
    break;

  case 6:
    color.g = 1.0;
    color.b = 0.5;
    color.r = 0.5;
    break;

  case 7:
    color.r = 1.0;
    color.b = 0.5;
    color.g = 0.5;
    break;
  case 8:
    color.b = 1.0;
    color.g = 0.5;
    color.r = 0.5;
    break;

  case 9:
    color.g = 1.0;
    color.b = 1.0;
    color.r = 0.5;
    break;
  }

  i = num % (n_colors*2);
  if (i >= n_colors) {
    color.g *= 0.5;
    color.b *= 0.5;
    color.r *= 0.5;
  }

  if (num < 0)
    color.r = color.b = color.g = 1.0;

  return color;
}

