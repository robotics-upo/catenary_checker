#include <catenary_checker/catenary_checker_manager.h>
#include <chrono>

using namespace std::chrono;

CatenaryCheckerManager::CatenaryCheckerManager(std::string node_name_)
{
  nh.reset(new ros::NodeHandle("~"));
	std::string node_name = "catenaryChecker_" + node_name_;

	grid_3D = new Grid3d(node_name);

	cc = new catenaryChecker(nh);
    
  point_cloud_sub_ = nh->subscribe<sensor_msgs::PointCloud2>
    ("/octomap_point_cloud_centers", 1, &CatenaryCheckerManager::PointCloudCallback, this);
}

void CatenaryCheckerManager::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    nn_obs.setInput(*msg);
    cc->getPointCloud(msg);
    cc->getDataForDistanceinformation(grid_3D, msg, use_distance_function);
    ROS_INFO(PRINTF_YELLOW "CatenaryCheckerManager::PointCloudCallback: Received Point Cloud");
}
 
void CatenaryCheckerManager::Init(double dist_cat_, double l_cat_max_, double ws_z_min_,
                                  double step_, bool use_parable_,
                                  bool use_distance_function_, bool use_both_)
{
  distance_catenary_obstacle = dist_cat_;
  length_tether_max = l_cat_max_;
  ws_z_min = ws_z_min_;
  step = step_; 
  catenary_state = false;
  use_parable = use_parable_;
  use_both = use_both_;
	use_distance_function = use_distance_function_;

  if (use_both) {
    ROS_INFO(PRINTF_GREEN "CatenaryCheckerManager: Using both methods");
  } else if (use_parable) {
    ROS_INFO(PRINTF_GREEN "CatenaryCheckerManager: Using PARABLE METHOD");
  } else
    ROS_INFO(PRINTF_GREEN "CatenaryCheckerManager: Using NUMERICAL METHOD from Ceres Solver");

	if (use_distance_function_)
    ROS_INFO(PRINTF_GREEN "CatenaryCheckerManager: Using DISTANCE FUNCTION");
  else
    ROS_INFO(PRINTF_GREEN "CatenaryCheckerManager: Using KDTree");
}

bool CatenaryCheckerManager::SearchCatenary(const geometry_msgs::Point &pi_,
                                            const geometry_msgs::Point &pf_,
                                            std::vector<geometry_msgs::Point> &pts_c_)
{
  bool is_found;
  pts_c_.clear();
  if (use_parable || use_both) {
    float a = cc->precomputing_time;
    auto t1 = high_resolution_clock::now();
    is_found = checkStraightCatenary(pi_, pf_, pts_c_, 0.1);
    if (!is_found) {
      pts_c_.clear();
      is_found = cc->analyticalCheckCatenary(pi_, pf_, pts_c_);
      if(is_found)
        {
          min_dist_obs_cat = cc->min_dist_obs_cat;
          length_cat_final = cc->length_cat;
        } else {
        min_dist_obs_cat = -1.0;
        length_cat_final = -1.0;
      }
    }
    auto t2 = high_resolution_clock::now();
    duration<float>fp_s = t2 - t1;
    float delta_t = fp_s.count();


    //Check for precomputing time
    if (a < 0.0 && cc->precomputing_time > 0.0) {
      planes_precomputing_time = a = cc->precomputing_time;
      delta_t -= a;
    } 
    execution_times_parable.push_back(delta_t);
    results_parable.push_back(length_cat_final);
  }
  if (!use_parable || use_both) {
    auto t1 = high_resolution_clock::now();
    is_found = NumericalSolutionCatenary(pi_, pf_ ,pts_c_);

    float length_cat = 0.0;
    for (unsigned int i = 1; i < pts_c_.size() && is_found;i++) {
      auto &p1 = pts_c_[i];
      auto &p0 = pts_c_[i - 1];
      length_cat += sqrt( pow(p1.x - p0.x, 2.0) + pow(p1.y - p0.y, 2.0) +
                          pow(p1.z - p0.z, 2.0));
    }
    if (!is_found) {
      length_cat = -1.0;
    }
    auto t2 = high_resolution_clock::now();
    duration<float>fp_s = t2 - t1;
    float delta_t = fp_s.count();
    execution_times_bisection.push_back(delta_t);

    results_bisection.push_back(length_cat);
  }

	return is_found;
}

bool CatenaryCheckerManager::checkStraightCatenary(const geometry_msgs::Point &A,
                                                    const geometry_msgs::Point &B,
                                                    std::vector<geometry_msgs::Point> &p,
                                                    double step) {
  length_cat_final = length(A, B);

  int steps = ceil(length_cat_final / step);
  double step_x = B.x - A.x;
  double step_y = B.y - A.y;
  double step_z = B.z - A.z;
  geometry_msgs::Point C = A;

  min_dist_obs_cat = 1e10;
  p.resize(steps);

  for (int i = 0; i < steps && min_dist_obs_cat > distance_catenary_obstacle; i++) {
    p[i] = C;
    double curr_dist = getPointDistanceFullMap(true, C);
    min_dist_obs_cat = std::min(curr_dist, min_dist_obs_cat);

    C.x += step_x;
    C.y += step_y;
    C.z += step_z;
  }

  return min_dist_obs_cat > distance_catenary_obstacle;
}

bool CatenaryCheckerManager::checkCatenary(const geometry_msgs::Point &A,
                                           const geometry_msgs::Point &B,
                                           double l) {
  bool just_one_axe = bc.configBisection(l, A.x, A.y, A.z, B.x, B.y, B.z,
                                         false);
  std::vector<geometry_msgs::Point> points_catenary_;
  bc.getPointCatenary3D(points_catenary_);
  return checkPoints(points_catenary_); // TODO: include parable approx
}

bool CatenaryCheckerManager::checkPoints(const std::vector<geometry_msgs::Point>
                                         &points_catenary_) {
  double min_dist_obs = 1e10;
  for (auto &p:points_catenary_) {
    double dist_cat_obs = getPointDistanceFullMap(use_distance_function, p);

    if (min_dist_obs > dist_cat_obs){
      min_dist_obs = dist_cat_obs;
      if (min_dist_obs < distance_catenary_obstacle)
        return false;
    }
  }

  return true;
}

bool CatenaryCheckerManager::NumericalSolutionCatenary(const geometry_msgs::Point &p_reel_, const geometry_msgs::Point &p_final_, std::vector<geometry_msgs::Point> &points_catenary_)
{
	double dist_init_final_ = sqrt(pow(p_reel_.x - p_final_.x,2) +
                                 pow(p_reel_.y - p_final_.y,2) +
                                 pow(p_reel_.z - p_final_.z,2));
	double delta_ = 0.0;	//Initial Value
	bool check_catenary = true;
	bool founded_catenary = false;
	bool increase_catenary;
	double length_catenary_;
	int n_points_cat_dis_;
	double security_dis_ca_ = distance_catenary_obstacle;

	double coef_safety;
	if (dist_init_final_ < 4.0)
		coef_safety = 1.010;
	else
		coef_safety = 1.005;

	do {
		increase_catenary = false;
		points_catenary_.clear();
		length_catenary_ = dist_init_final_* (coef_safety + delta_);
		if (length_catenary_ > length_tether_max){
			check_catenary = false;
			break;
		}
		bool just_one_axe = bc.configBisection(length_catenary_, p_reel_.x, p_reel_.y,
                                           p_reel_.z, p_final_.x, p_final_.y, p_final_.z,
                                           false);
		bc.getPointCatenary3D(points_catenary_);

		double d_min_point_cat = 100000;
		if (points_catenary_.size() > 2){
      // parameter to ignore collsion points in the begining and in the end of catenary
			n_points_cat_dis_ = ceil(1.5*ceil(length_catenary_)); 
			if (n_points_cat_dis_ < 5)
				n_points_cat_dis_ = 5;
			for (size_t i = 0 ; i < points_catenary_.size() ; i++){
				geometry_msgs::Point point_cat;
				geometry_msgs::Vector3 p_in_cat_;
				if (points_catenary_[i].z < ws_z_min*step + ((1*step)+security_dis_ca_)){
					check_catenary = false;
					break;
				}
				if ((i > n_points_cat_dis_ ) ){
					auto &p_in_cat_ = points_catenary_[i];
					double dist_cat_obs = getPointDistanceFullMap(use_distance_function,
                                                        p_in_cat_);

					if (d_min_point_cat > dist_cat_obs){
						min_dist_obs_cat = dist_cat_obs;
						d_min_point_cat = dist_cat_obs;
					}
					if (dist_cat_obs < security_dis_ca_){
						delta_ = delta_ + 0.005;
						increase_catenary = true;
						break;
					}
				}
				point_cat.x = points_catenary_[i].x;
				point_cat.y = points_catenary_[i].y;
				point_cat.z = points_catenary_[i].z;
			}
			if (check_catenary && !increase_catenary){
				founded_catenary = true;
				check_catenary = false;
				length_cat_final = length_catenary_;
			}
		} else {
			check_catenary = false;
		}
	}while (check_catenary);
	
	if (!founded_catenary ){//In case not feasible to find catenary
		length_cat_final = -1.0;	
		min_dist_obs_cat = -1.0;
		points_catenary_.clear();
	}
	catenary_state = founded_catenary;

	return founded_catenary;
}

double CatenaryCheckerManager::getPointDistanceFullMap(bool use_dist_func_,
                                                       geometry_msgs::Point p_)
{
	double dist;
	Eigen::Vector3d obs_, pos_;

	if(use_dist_func_){
		bool is_into_ = grid_3D->isIntoMap(p_.x,p_.y,p_.z);
		if(is_into_)
			dist =  grid_3D->getPointDist((double)p_.x,(double)p_.y,(double)p_.z) ;
		else
			dist = -1.0;
	} else {
		pos_.x() = p_.x;
		pos_.y() = p_.y; 
		pos_.z() = p_.z; 
		obs_ = nn_obs.nearestObstacleMarsupial(nn_obs.kdtree, pos_, nn_obs.obs_points);
		dist = sqrt(pow(obs_.x()-pos_.x(),2) + pow(obs_.y()-pos_.y(),2) +
                pow(obs_.z()-pos_.z(),2));
	}

	return dist;
}

bool CatenaryCheckerManager::exportStats(const std::string &filename) const {
  ofstream ofs(filename.c_str());
  size_t s = std::max(execution_times_bisection.size(), execution_times_parable.size());
  ofs << "Precomputing time: " << cc->precomputing_time << "\n";
  for (size_t i = 0; i < s; i++) {
    if (use_parable || use_both) {
      ofs << execution_times_parable[i] << "\t" << results_parable[i] << "\t";
    }
    if (!use_parable || use_both) {
      ofs << execution_times_bisection[i] << "\t" << results_bisection[i] << "\t";
    }
    ofs << "\n";
  }
  return true;
}

CatenaryCheckerManager::~CatenaryCheckerManager() {
}
