#include <catenary_checker/preprocessed_scenario.hpp>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <pcl_conversions/pcl_conversions.h>


using namespace pcl;
using namespace std;

PreprocessedScenario::PreprocessedScenario(const std::string &filename) {
  ros::NodeHandle nh, pnh("~");

  _pub = nh.advertise<sensor_msgs::PointCloud2>("slices", 1, true);
  _pub_marker = nh.advertise<visualization_msgs::MarkerArray>("scenarios", 2, true);
  _sub = nh.subscribe("point_cloud", 1, &PreprocessedScenario::PC_Callback, this);

  _filename = filename;


  if (!loadScenario(filename)) {
    pnh.param("ws_x_min", _min.x, 0.0f);
    pnh.param("ws_y_min", _min.y, 0.0f);
    pnh.param("ws_x_max", _max.x, 10.0f);
    pnh.param("ws_y_max", _max.y, 10.0f);
    pnh.param("ws_z_max", _max_z, 10.0f);
    pnh.param("n_theta", _n_theta, 60);
    pnh.param("plane_dist", _plane_dist, 0.1f);

    pnh.param("db_min_points", _db_min_points, 20);
    pnh.param("db_epsilon", _db_epsilon, 0.05f);

    ROS_INFO("Started Preprocessed Scenario from ROS. ");
    ROS_INFO("min = %s \t max = %s", _min.toString().c_str(),_max.toString().c_str());
    ROS_INFO("n_theta = %d\tplane_dist = %f", _n_theta, _plane_dist);
  } else {
    ROS_INFO("PreprocessedScenario::Got the scenarios from file %s. Publishing.", filename.c_str());
    sleep(2);
  }
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
  double gamma = atan2(delta_x, delta_y); // We want the complementary angle (hence x,y)
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

float PreprocessedScenario::checkCatenary(const pcl::PointXYZ &A, const pcl::PointXYZ &B)
{
  double inc_x = B.x - A.x;
  double inc_y = B.y - A.y;
  double yaw = atan(inc_y/inc_x);

  float ret_val = -1.0;

  // Get the theta
  int i = round((yaw + M_PI * 0.5) / (_n_theta + 1));
  i = i % _n_theta;

  // Then get the plane

  const auto &scenes = _scenarios[i];
  if (scenes.size() > 0) {
    const Scenario &s_ = scenes[0];
    float min_dist = fabs(s_.plane.getSignedDistance(A));
    int i = 0;
    int j = 0;
    for (const auto &s:scenes) {
      auto curr_dist = s.plane.getSignedDistance(A);
      if (curr_dist < min_dist) {
        j = i;
        curr_dist = min_dist;
      }
      i++;
    }


    const Scenario &scen = scenes[j];
    _last_plane = scen.plane;
    _pa = scen.to2D(A);
    _pb = scen.to2D(B);
    _parabola.reset();

    if (_parabola.approximateParabola(scen, _pa, _pb)) {
      ret_val = _parabola.getLength(_pa.x, _pb.x);
      // ROS_INFO("PreprocessedScenario::checkCatenary --> could get the parabola. Length = %f", ret_val);

    } else {
      // ROS_INFO("PreprocessedScenario::checkCatenary --> could NOT get the parabola.");
    }
  }

  return ret_val;
}

namespace fs = std::filesystem;

// Function that exports scenarios --> I think that it would be nice to pack it into several files, not just one, an then compress it!
// TODO: End and test!
bool PreprocessedScenario::exportScenario() const {
  bool ret_val = true;

  // Ensure that we have a proper scenario
  if (_scenarios.size() != _n_theta) {
    return false;
  }


  fs::path original_path = (fs::current_path());
  fs::current_path(fs::temp_directory_path());

  string name = getFilename(_filename);

  if (fs::is_directory(name)) {
    fs::remove_all(name);
  }
  if (fs::create_directories(name)) {
    fs::current_path(name);
     int i = 0;
    // Export the metadata
    ofstream metadata(string(name + ".yaml").c_str());
    YAML::Emitter e;
    e << YAML::BeginMap;
    e << YAML::Key << "min" << YAML::Value << _min;
    e << YAML::Key << "max" << YAML::Value << _max;
    e << YAML::Key << "max_z" << YAML::Value << _max_z;
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
      cerr << "Could not create: " << _filename << "\t Failure:" << e.what() << endl;
    }
  }
  fs::current_path(fs::temp_directory_path());


  ostringstream oss;
  oss << "tar czf " << _filename << " " << name;
  ROS_INFO("Executing: %s ", oss.str().c_str());

  if (system(oss.str().c_str()) == 0) {
    ROS_INFO("File %s created successfully", _filename.c_str());
  }

  fs::current_path(original_path);

  return ret_val;
}

bool PreprocessedScenario::loadScenario(const std::string &file) {
  _scenarios.clear();

  bool ret_val = true;

  fs::path original_path = fs::current_path();
  fs::current_path(fs::temp_directory_path());

  ostringstream oss;
  oss << "tar xf " << file;
  cout << "Executing command: \"" << oss.str() << "\"" << endl;

  if (system(oss.str().c_str()) == 0) {
    ROS_INFO("File %s decompressed OK. ", file.c_str());
  } else {
    ROS_INFO("Could not decompress %s. Calculating scenarios.", file.c_str());
    return false;
  }

  string f = getFilename(file); // gets the filename without .tar.gz and without /
  ROS_INFO("Filename: %s", f.c_str());

  fs::current_path(f);

  if (getMetadata(f+".yaml")) {
    for (int i = 0; i < _n_theta; i++) {
      vector<Scenario> curr_vec;
      fs::current_path(std::to_string(i));

      
      for (int j = 0;fs::is_regular_file(std::to_string(j)); j++) {
        Scenario s;
        if (s.loadScenario(std::to_string(j))) {
          curr_vec.push_back(s);
        }
      }
      ROS_INFO("Loaded angle %d. Number of scenarios %d", i, static_cast<int>(curr_vec.size()));
      _scenarios.push_back(curr_vec);
      fs::current_path("..");
    }
    ROS_INFO("Loaded scenario. Number of angles: %d", _n_theta);
  } else {
    ret_val = false;
  }

  fs::current_path(original_path);

  return ret_val;
}

bool PreprocessedScenario::getMetadata(const std::string &f) {
  bool ret_val = true;

  try {
    ifstream ifs(f.c_str());

    YAML::Node y = YAML::Load(ifs);

    _min = Point2D(y["min"]);
    _max = Point2D(y["max"]);
    _max_z = y["max_z"].as<float>();
    _plane_dist = y["plane_dist"].as<float>();
    _n_theta = y["n_theta"].as<int>();
    _db_min_points = y["db_min_points"].as<int>();
    _db_epsilon = y["db_epsilon"].as<float>();
  } catch (exception &e) {
    cerr << "PreprocessedScenario::getMetadata --> exception when loading metadata.";
    cerr << "Content: " << e.what() << endl;
    ret_val = false;
  }

  return ret_val;
}

string PreprocessedScenario::getFilename(const string &f) const {
  string ret;

  auto start_ = f.rfind('/');
  auto end_ = f.rfind(".tar.gz");

  if (start_ == std::string::npos) {
    start_ = 0;
  } else {
    start_++;
  }

  if (end_ == std::string::npos) {
    end_ = f.size();
  }

  return f.substr(start_, end_ - start_);
}

void PreprocessedScenario::PC_Callback(const sensor_msgs::PointCloud2::ConstPtr &pc) {
  if (_scenarios.size() > 0) {
    return; // ALready computed
  }
  pcl::PointCloud<pcl::PointXYZ> pcl_pc;

  pcl::fromROSMsg(*pc, pcl_pc);
  ROS_INFO("Received PointCloud. Starting preprocessing. Height, Width: %d, %d", pc->height, pc->width);
  precompute(pcl_pc);

  ROS_INFO("Exporting scenario. Filename %s", _filename.c_str());
  exportScenario();
}

void PreprocessedScenario::publishScenarios(unsigned int scen) {
  if (_scenarios.size() > scen) {
    ROS_INFO("Publishing scenario set %u.", scen);

    int i = 0;
    int total = _scenarios[scen].size();
    for (auto &x:_scenarios[scen]) {
      if (x.getTotalPoints() > 200) {
        float intensity = 0.5f + 0.5f/(static_cast<float>(i) / static_cast<float>(total));
        _pub.publish(x.toPC("map", i, intensity));
        _pub_marker.publish(x.toMarkerArray("map", i++));
      }
    }
  }
}
