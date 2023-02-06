#include <catenary_checker/catenary_checker_manager.h>


CatenaryCheckerManager::CatenaryCheckerManager(std::string node_name_)
{
    nh.reset(new ros::NodeHandle("~"));
	std::string node_name = "catenaryChecker_" + node_name_;

    use_distance_function = false;

	grid_3D = new Grid3d(node_name);
	cc = new catenaryChecker(nh);
    
    point_cloud_sub_ = nh->subscribe<sensor_msgs::PointCloud2>("/octomap_point_cloud_centers", 1, &CatenaryCheckerManager::PointCloudCallback, this);
}

void CatenaryCheckerManager::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    nn_obs.setInput(*msg);
	cc->getPointCloud(msg);
	cc->getDataForDistanceinformation(grid_3D, msg, use_distance_function);
    ROS_INFO(PRINTF_YELLOW "CatenaryCheckerManager::PointCloudCallback: Received Point Cloud");
}
 
void CatenaryCheckerManager::Init(double dist_cat_, double l_cat_max_, double ws_z_min_, double step_, bool use_analytical_method_)
{
    distance_catenary_obstacle = dist_cat_;
    length_tether_max = l_cat_max_;
    ws_z_min = ws_z_min_;
    step = step_; 
    catenary_state = false;
    use_analytical_method = use_analytical_method_;
	
	if (use_analytical_method)
        ROS_INFO(PRINTF_GREEN "Catenary Checker Manager: Using ANALYTICAL METHOD");
    else
        ROS_INFO(PRINTF_GREEN "Catenary Checker Manager: Using NUMERICAL METHOD from Ceres Solver");
	
}

bool CatenaryCheckerManager::SearchCatenary(const geometry_msgs::Point &pi_, const geometry_msgs::Point &pf_, std::vector<geometry_msgs::Point> &pts_c_)
{
   bool is_founded;
   pts_c_.clear();
    if (use_analytical_method){
        if(is_founded = cc->analyticalCheckCatenary(pi_, pf_, pts_c_)){
			min_dist_obs_cat = cc->min_dist_obs_cat;
			length_cat_final = cc->length_cat;
		}
		else{
			min_dist_obs_cat = -1.0;
			length_cat_final = -1.0;
		}
		// for (size_t i = 0; i < pts_c_.size() ; i++){
		// 	printf("CatenaryCheckerManager::SearchCatenary: pts_c_[%f %f %f]\n", pts_c_[i].x,pts_c_[i].y,pts_c_[i].z);
		// }
		// printf("is_founded catenary = %s \t\n\n",is_founded?"true":"false");
    }
	else
        is_founded = NumericalSolutionCatenary(pi_, pf_ ,pts_c_);

	return is_founded;
}

bool CatenaryCheckerManager::NumericalSolutionCatenary(const geometry_msgs::Point &p_reel_, const geometry_msgs::Point &p_final_, std::vector<geometry_msgs::Point> &points_catenary_)
{
	double dist_init_final_ = sqrt(pow(p_reel_.x - p_final_.x,2) + pow(p_reel_.y - p_final_.y,2) + pow(p_reel_.z - p_final_.z,2));
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

	do{
		increase_catenary = false;
		points_catenary_.clear();
		length_catenary_ = dist_init_final_* (coef_safety + delta_);
		if (length_catenary_ > length_tether_max){
			check_catenary = false;
			break;
		}
		bool just_one_axe = bc.configBisection(length_catenary_, p_reel_.x, p_reel_.y, p_reel_.z, p_final_.x, p_final_.y, p_final_.z, false);
		bc.getPointCatenary3D(points_catenary_);

		double d_min_point_cat = 100000;
		if (points_catenary_.size() > 2){
			n_points_cat_dis_ = ceil(1.5*ceil(length_catenary_)); // parameter to ignore collsion points in the begining and in the end of catenary
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
					p_in_cat_.x = points_catenary_[i].x;
					p_in_cat_.y = points_catenary_[i].y;
					p_in_cat_.z = points_catenary_[i].z;
					
					double dist_cat_obs = getPointDistanceFullMap(use_distance_function, p_in_cat_);

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
		}
		else{
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

double CatenaryCheckerManager::getPointDistanceFullMap(bool use_dist_func_, geometry_msgs::Vector3 p_)
{
	double dist;
	Eigen::Vector3d obs_, pos_;

	if(use_dist_func_){
		bool is_into_ = grid_3D->isIntoMap(p_.x,p_.y,p_.z);
		if(is_into_)
			dist =  grid_3D->getPointDist((double)p_.x,(double)p_.y,(double)p_.z) ;
		else
			dist = -1.0;
	}
	else{
		pos_.x() = p_.x;
		pos_.y() = p_.y; 
		pos_.z() = p_.z; 
		obs_ = nn_obs.nearestObstacleMarsupial(nn_obs.kdtree, pos_, nn_obs.obs_points);
		dist = sqrt(pow(obs_.x()-pos_.x(),2) + pow(obs_.y()-pos_.y(),2) + pow(obs_.z()-pos_.z(),2));
	}

	return dist;
}