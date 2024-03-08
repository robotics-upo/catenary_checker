#include <catenary_checker/preprocessed_scenario.hpp>
#include <cmath>
#include <fstream>
#include <filesystem>

using namespace pcl;
using namespace std;

PreprocessedScenario::PreprocessedScenario(Point2D min, Point2D max,
                                           int n_theta, float plane_dist,
                                           int dbscan_min_points, float dbscan_epsilon) {
  _min = min;
  _max = max;
  _n_theta = n_theta;
  _plane_dist = plane_dist;
  _db_min_points = dbscan_min_points;
  _db_epsilon = dbscan_epsilon;
}

PreprocessedScenario::PreprocessedScenario(ros::NodeHandle &pnh) {
  pnh.param("min_x", _min.x, 0.0f);
  pnh.param("min_y", _min.y, 0.0f);
  pnh.param("max_x", _max.x, 10.0f);
  pnh.param("max_y", _max.y, 10.0f);
  pnh.param("max_z", _max_z, 10.0f);
  pnh.param("n_theta", _n_theta, 10);
  pnh.param("plane_dist", _plane_dist, 0.2f);

  pnh.param("db_min_points", _db_min_points, 20);
  pnh.param("db_epsilon", _db_epsilon, 0.05f);

  ROS_INFO("Started Preprocessed Scenario from ROS. ");
  ROS_INFO("min = %s \t max = %s", _min.toString().c_str(),_max.toString().c_str());
  ROS_INFO("n_theta = %d\tplane_dist = %f", _n_theta, _plane_dist);
}

void PreprocessedScenario::precompute(const PointCloud<PointXYZ> &pc) {
  // We have to sample only Pi (one plane goes to a direction and its opposite)
  const float increment = M_PI / static_cast<float>(_n_theta + 1);

  _scenarios.clear();
  _problems.clear();

  float angle = -M_PI * 0.5;
  // First we sample the angle

  for (int i = 0; i < _n_theta; i++, angle += increment) {
    vector<Scenario> planes;    
    vector<TwoPoints> ps = getProblemsTheta(angle);

    // For each problem get the obstacles associated
    for (auto &x:ps) {
      PointXYZ A(x.first.x, x.first.y, _max_z);
      PointXYZ B(x.second.x, x.second.y, _max_z);
      auto scene = PC2Obstacles(A,
                                B,
                                pc,
                                _plane_dist,
                                _db_min_points,
                                _db_epsilon);
      planes.push_back(scene);
    }
    _scenarios.push_back(planes);
    _problems.push_back(ps);
  }
}

//! @brief Gets the problems to sample the workspace with vertical planes in the theta direction
//! @param theta The direction (x axis)
//! @param min Minimum x and y
//! @param max Maximum x and y
//! @param dist Distance between planes
vector<TwoPoints> PreprocessedScenario::getProblemsTheta(double theta) const {
  vector<TwoPoints> ret;

  double delta_x = _max.x - _min.x;
  double delta_y = _max.y - _min.y;
  double diag = sqrt(delta_x*delta_x + delta_y*delta_y);

  double s_theta = sin(theta);
  double c_theta = cos(theta);

  Point2D S, E, A, B; // Starting point
  S = _min;
  E = _max;
  if (theta >= 0.0) {
    S.y = _max.y;
    E.y = _min.y;
  }
  double gamma = atan2(delta_x, delta_y); // Note that we want the complementar (hence x,y)
  int n_planes = round(diag * cos(gamma - fabs(theta)) / _plane_dist);
  double inc_c = _plane_dist;
  if (!signbit(theta) || theta == 0.0) {
    // In these cases we should go the other way round
    inc_c *= -1.0;
  }
  double curr_c = -s_theta * S.x + c_theta * S.y + 0.5 *inc_c;
  
  for (int i = 0; i < n_planes; i++, curr_c += inc_c) {
    // Get the y_c for x_min
    A.x = S.x;
    A.y = (curr_c + s_theta * S.x) / c_theta;
    if (A.y < _min.y || A.y > _max.y || isnan(A.y)) {
      A.y = E.y;
      A.x = (c_theta * E.y - curr_c) / s_theta;
    }

    // Get the B.x for E.y
    B.y = S.y;
    B.x = (c_theta * B.y - curr_c) / s_theta;
    if (B.x < _min.x || B.x > _max.x || isnan(B.x)) {
      B.x = E.x;
      B.y = (curr_c + s_theta * E.x) / c_theta;
    }
    ret.push_back(TwoPoints(A,B));
  }

  return ret;
}

float PreprocessedScenario::checkCatenary(const pcl::PointXYZ &A, const pcl::PointXYZ &B
                                          , double max_length) const 
{
  double inc_x = B.x - A.x;
  double inc_y = B.y - A.y;
  double yaw = atan(inc_y/inc_x);

  // Get the theta
  int i = (yaw + M_PI * 0.5) / (_n_theta + 1);

  // Then get the plane
  // TODO: Complete! (get the scenario and call the parabol method with that scenario)
  return -1.0;
}

namespace fs = std::filesystem;

// Function that exports scenarios --> I think that it would be nice to pack it into several files, not just one, an then compress it!
// TODO: End and test!
bool PreprocessedScenario::exportScenarios(const std::string &name) const {
  bool ret_val = true;

  // Ensure that we have a proper scenario
  if (_scenarios.size() != _n_theta) {
    return false;
  }


  fs::path original_path = (fs::current_path());
  fs::current_path(fs::temp_directory_path());


  if (fs::create_directory(name)) {
    fs::current_path(name);
    int i = 0;
    // Export the metadata
    ofstream metadata(string(name + ".yaml").c_str());
    YAML::Emitter e;
    e << YAML::BeginMap;
    e << YAML::Key << "min" << YAML::Value << _min;
    e << YAML::Key << "max" << YAML::Value << _max;
    e << YAML::Key << "n_theta" << YAML::Value << _n_theta;
    e << YAML::Key << "plane_dist" << YAML::Value << _plane_dist;
    e << YAML::Key << "db_min_points" << YAML::Value << _db_min_points;
    e << YAML::Key << "db_epsilon" << YAML::Value << _db_epsilon;
    metadata << e.c_str() << endl;
    metadata.close();

    try {
      // Save the scenarios
      for (int i = 0; i < _n_theta; i++) {
        string curr_dir = to_string(i);
        fs::create_directory(curr_dir);
        fs::current_path(curr_dir);
        auto &curr_vec = _scenarios[i];
        int j = 0;

        for (auto &x:curr_vec) {
          string curr_file = to_string(j++);
          ofstream ofs(curr_file);
          YAML::Emitter e;
          e << x;
          ofs << e.c_str()<<endl;
        }
        fs::current_path(".."); 
      }
    } catch(ios_base::failure &e) {
      cerr << "Could not create: " << name << "\t Failure:" << e.what() << endl;
    }
  }
  fs::current_path(original_path);


  return ret_val;
}
