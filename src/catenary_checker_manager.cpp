#include <catenary_checker/catenary_checker_manager.h>


CatenaryCheckerManager::CatenaryCheckerManager(std::string node_name_)
{
    nh.reset(new ros::NodeHandle("~"));
	std::string node_name = "catenaryChecker_" + node_name_;
	std::cout << std::endl << "Starting clar CatenaryCheckManager from " << node_name << std::endl;

	cc = new catenaryChecker(nh);
    
    point_cloud_sub_ = nh->subscribe<sensor_msgs::PointCloud2>("/octomap_point_cloud_centers", 1, &CatenaryCheckerManager::PointCloudCallback, this);
    point_cloud_ugv_obs_sub_ = nh->subscribe<sensor_msgs::PointCloud2>("/region_growing_obstacles_pc_map", 1, &CatenaryCheckerManager::PointCloudObstaclesCallback, this);
}

void CatenaryCheckerManager::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	cc->getPointCloud(msg);
	cc->getDataForDistanceinformation(grid_3D, msg, use_distance_function);
    ROS_INFO(PRINTF_YELLOW "CatenaryCheckerManager::PointCloudCallback: Received Point Cloud");
}

void CatenaryCheckerManager::PointCloudObstaclesCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    nn_obs.setInput(*msg);
    ROS_INFO(PRINTF_YELLOW "CatenaryCheckerManager::PointCloudObstaclesCallback: Received Point Cloud UGV Obstacles");
}
 
void CatenaryCheckerManager::Init(Grid3d *grid_3D_, double d_obs_tether_, double d_obs_ugv_, double d_obs_uav_, double l_cat_max_, double ws_z_min_, 
								double step_, bool use_parabola_, bool use_distance_function_, geometry_msgs::Vector3 p_reel_ugv_, bool j_l_o_s_, bool use_catenary_as_tether_)
{
	ROS_INFO(PRINTF_GREEN "Initializing CatenaryCheckerManager");
	
	grid_3D = grid_3D_;
    length_tether_max = l_cat_max_;
    ws_z_min = ws_z_min_;
    step = step_; 
    catenary_state = false;
    use_parabola = use_parabola_;
	use_distance_function = use_distance_function_;
	just_line_of_sigth = j_l_o_s_;

	distance_obstacle_ugv = d_obs_ugv_;
	distance_obstacle_uav = d_obs_uav_;
	distance_tether_obstacle = d_obs_tether_;
	use_catenary_as_tether = use_catenary_as_tether_;

	p_reel_ugv = p_reel_ugv_;

	if (use_parabola)
        ROS_INFO(PRINTF_GREEN "CatenaryCheckerManager: Using PARABOLA METHOD");
    else
        ROS_INFO(PRINTF_GREEN "CatenaryCheckerManager: Using NUMERICAL METHOD from Ceres Solver");

	if (use_distance_function_)
        ROS_INFO(PRINTF_GREEN "CatenaryCheckerManager: Using DISTANCE FUNCTION");
    else
        ROS_INFO(PRINTF_GREEN "CatenaryCheckerManager: Using KDTree");
}

bool CatenaryCheckerManager::SearchCatenary(const geometry_msgs::Vector3 &pi_, const geometry_msgs::Vector3 &pf_, std::vector<geometry_msgs::Vector3> &pts_c_)
{
   bool is_founded;
   pts_c_.clear();
   	if(just_line_of_sigth){
		is_founded = computeStraight(pi_, pf_ ,pts_c_);
	}else{ // Just to check a straigth line 
		if (use_parabola){
			if(is_founded = cc->analyticalCheckCatenary(pi_, pf_, pts_c_)){
				min_dist_obs_cat = cc->min_dist_obs_cat;
				length_cat_final = cc->length_cat;
			}
			else{
				min_dist_obs_cat = -1.0;
				length_cat_final = -1.0;
			}
		}
		else
			is_founded = NumericalSolutionCatenary(pi_, pf_ ,pts_c_);
	}

	return is_founded;
}

bool CatenaryCheckerManager::computeStraight(const geometry_msgs::Vector3 &p_reel_, const geometry_msgs::Vector3 &p_final_, std::vector<geometry_msgs::Vector3> &points_catenary_)
{ 
	bool founded_catenary = true;
	points_catenary_.clear();
	double dist_init_final_ = 1.005 * sqrt(pow(p_reel_.x - p_final_.x,2) + pow(p_reel_.y - p_final_.y,2) + pow(p_reel_.z - p_final_.z,2));
	length_cat_final = dist_init_final_;

	double dx_ = p_final_.x - p_reel_.x ;
	double dy_ = p_final_.y - p_reel_.y ;
	double dz_ = p_final_.z - p_reel_.z ;
	int num_point_catenary = round( (double)10.0 * fabs(dz_));

    double x_step = dx_ / (double) num_point_catenary;
    double y_step = dy_ / (double) num_point_catenary;
    double z_step = dz_ / (double) num_point_catenary;

	// printf("GetParabolaParameter Straigth Tether: d_=[%.3f %.3f %.3f], num_point_catenary= %i , step=[%.3f %.3f %.3f]\n", dx_,dy_,dz_, num_point_catenary, x_step, y_step,z_step);

    for(int i=0; i < num_point_catenary ; i++)
    {       
        geometry_msgs::Vector3 p_;
        p_.x = (p_reel_.x + x_step* (double)i);
        p_.y = (p_reel_.y + y_step* (double)i);
        p_.z = (p_reel_.z + z_step* (double)i);    
        points_catenary_.push_back(p_);
		double dist_cat_obs = getPointDistanceObstaclesMap(use_distance_function, p_);

		if (dist_cat_obs < distance_tether_obstacle){
			founded_catenary = false;
			length_cat_final = -1.0;
			// std::cout << "CatenaryCheckerManager::computeStraight :	["<< i <<"] dist_collision = " << dist_cat_obs << " p_=["<<p_.x << "," << p_.y << "," << p_.z  <<"]" << std::endl;
			break;
		}
    }

	// printf("GetParabolaParameter Straigth Tether: Founded tether \n");
	return founded_catenary;
}

bool CatenaryCheckerManager::NumericalSolutionCatenary(const geometry_msgs::Vector3 &p_reel_, const geometry_msgs::Vector3 &p_final_, std::vector<geometry_msgs::Vector3> &points_catenary_)
{
	double dist_init_final_ = sqrt(pow(p_reel_.x - p_final_.x,2) + pow(p_reel_.y - p_final_.y,2) + pow(p_reel_.z - p_final_.z,2));
	double delta_ = 0.0;	//Initial Value
	bool check_catenary = true;
	bool founded_catenary = false;
	bool increase_catenary;
	double length_catenary_;
	int n_points_cat_dis_;
	param_cat_x0 = param_cat_y0 = param_cat_a = 0.0;

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
		bool just_one_axe = bc.configBisection(length_catenary_, p_reel_.x, p_reel_.y, p_reel_.z, p_final_.x, p_final_.y, p_final_.z);
		bc.getPointCatenary3D(points_catenary_, false);

		double d_min_point_cat = 100000;
		if (points_catenary_.size() > 2){
			n_points_cat_dis_ = ceil(1.5*ceil(length_catenary_)); // parameter to ignore collsion points in the begining and in the end of catenary
			if (n_points_cat_dis_ < 5)
				n_points_cat_dis_ = 5;
			for (size_t i = 0 ; i < points_catenary_.size() ; i++){
				geometry_msgs::Vector3 point_cat;
				geometry_msgs::Vector3 p_in_cat_;
				if (points_catenary_[i].z < ws_z_min*step + ((1*step)+distance_tether_obstacle)){
					check_catenary = false;
					break;
				}
				if ((i > n_points_cat_dis_ ) ){
					p_in_cat_.x = points_catenary_[i].x;
					p_in_cat_.y = points_catenary_[i].y;
					p_in_cat_.z = points_catenary_[i].z;
					
					double dist_cat_obs = getPointDistanceObstaclesMap(use_distance_function, p_in_cat_);

					if (d_min_point_cat > dist_cat_obs){
						min_dist_obs_cat = dist_cat_obs;
						d_min_point_cat = dist_cat_obs;
					}
					if (dist_cat_obs < distance_tether_obstacle){
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
		length_cat_final = -3.0;	
		min_dist_obs_cat = -1.0;
		points_catenary_.clear();
	}
	catenary_state = founded_catenary;

	param_cat_x0 = bc.Xc;
	param_cat_y0 = bc.Yc;
	param_cat_a = bc.c_value;

	return founded_catenary;
}

double CatenaryCheckerManager::getPointDistanceObstaclesMap(bool use_dist_func_, geometry_msgs::Vector3 p_)
{
	double dist;
	Eigen::Vector3d obs_, pos_;

	if(use_dist_func_){
		bool is_into_ = grid_3D->isIntoMap(p_.x,p_.y,p_.z);
		if(is_into_){
			// dist =  grid_3D->getPointDist((double)p_.x,(double)p_.y,(double)p_.z) ;
			TrilinearParams p = grid_3D->getPointDistInterpolation(p_.x, p_.y, p_.z);
            dist = p.a0 + p.a1*p_.x + p.a2*p_.y + p.a3*p_.z + p.a4*p_.x*p_.y + p.a5*p_.x*p_.z + p.a6*p_.y*p_.z + p.a7*p_.x*p_.y*p_.z;
		}
		else{
			dist = -1.0;
			// ROS_ERROR("CatenaryCheckerManager::getPointDistanceObstaclesMap : Point outside workspace");
		}
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

double CatenaryCheckerManager::getPointDistanceObstaclesMap(bool use_dist_func_, geometry_msgs::Vector3 p_, int pose_, string msg_)
{
	double dist, x, y, z;
	if(use_dist_func_){
		bool is_into_ = grid_3D->isIntoMap(p_.x,p_.y,p_.z);
		if(is_into_){
			x = (double)p_.x;
			y = (double)p_.y;
			z = (double)p_.z;
			TrilinearParams p = grid_3D->getPointDistInterpolation(x,y,z);
            dist = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;
			// dist =  grid_3D->getPointDist((double)p_.x,(double)p_.y,(double)p_.z) ;
		}
		else{
            std::cout << "\tThe agent " << msg_ << "[" << pose_ << "] is out of GRID["<<p_.x<< ", " << p_.y << ", " <<p_.z << "]"<< std::endl; 
			dist = -1.0;
        }
    }
	else{
		Eigen::Vector3d pos_, obs_;
		pos_.x() = p_.x;
		pos_.y() = p_.y; 
		pos_.z() = p_.z; 
		obs_= nn_obs.nearestObstacleMarsupial(nn_obs.kdtree, pos_, nn_obs.obs_points);
		if (!(pos_.z() > obs_.z()-0.6)) // - 0.06 is because UGV can drive above 6 cm obstacles
			dist = sqrt(pow(obs_.x()-pos_.x(),2) + pow(obs_.y()-pos_.y(),2) + pow(obs_.z()-pos_.z(),2));
		else	
			dist = 1.0; //For whole obstacle under or same altitud than UGV
	}
	return dist;
}

bool CatenaryCheckerManager::CheckStatusCollision(trajectory_msgs::MultiDOFJointTrajectory mt_, std::vector<double> ct_)
{
	bool ret_;

	geometry_msgs::Vector3 p_ugv_, p_uav_, p_reel_; 
	std::vector<geometry_msgs::Vector3> points_catenary_;
    double len_cat_, dist_;
	bisectionCatenary bc;
	count_ugv_coll = count_uav_coll = count_tether_coll = 0;
	v_pos_coll_tether.clear();

    std::cout << std::endl << "	CatenaryCheckerManager started: analizing collision status for the marsupial agents" << std::endl; 
    std::cout << "	                                   d_obs_ugv=" <<distance_obstacle_ugv << 
				 " , d_obs_uav="<< distance_obstacle_uav << 
				 " , d_obs_tether="<< distance_tether_obstacle<< std::endl; 
	
	int aux_coll_theter_ = 0;
	for (size_t i= 0 ; i <  mt_.points.size(); i++){
		p_ugv_.x = mt_.points.at(i).transforms[0].translation.x;
		p_ugv_.y = mt_.points.at(i).transforms[0].translation.y;
		p_ugv_.z = mt_.points.at(i).transforms[0].translation.z;
		p_uav_.x = mt_.points.at(i).transforms[1].translation.x;
		p_uav_.y = mt_.points.at(i).transforms[1].translation.y;
		p_uav_.z = mt_.points.at(i).transforms[1].translation.z;	
		// Catenary 
		len_cat_ = ct_[i];

		geometry_msgs::Vector3 p_;
		geometry_msgs::Quaternion q_;
		p_.x = p_ugv_.x;
		p_.y = p_ugv_.y;
		p_.z = p_ugv_.z;	//Move in Z to see the point over the map surface
		q_.x = mt_.points.at(i).transforms[0].rotation.x;
		q_.y = mt_.points.at(i).transforms[0].rotation.y;
		q_.z = mt_.points.at(i).transforms[0].rotation.z;
		q_.w = mt_.points.at(i).transforms[0].rotation.w;
		

		dist_ = getPointDistanceObstaclesMap(false, p_ugv_,i,"UGV");
        if (dist_ < distance_obstacle_ugv){
			count_ugv_coll++;
            std::cout << "      The agent UGV in the state = " << i << " is in COLLISION ["<< dist_ <<" mts to obstacle]" << std::endl; 
        }

        dist_ = getPointDistanceObstaclesMap(true, p_uav_,i,"UAV");
        if (dist_ < distance_obstacle_uav){
            count_uav_coll++;
		    std::cout << "      The agent UAV in the state = " << i << " idistance_tether_obstacles in COLLISION ["<< dist_ <<" mts to obstacle]" << std::endl; 
		}

		p_reel_ = getReelNode(p_,q_);

		points_catenary_.clear();
		bool just_one_axe = bc.configBisection(len_cat_, p_reel_.x, p_reel_.y, p_reel_.z, p_uav_.x, p_uav_.y, p_uav_.z);
		bc.getPointCatenary3D(points_catenary_, false);
		for (size_t j = 0 ; j < points_catenary_.size() ; j++ ) {
            dist_ = getPointDistanceObstaclesMap(true, points_catenary_[j],i,"TETHER") ;
            if( dist_ < distance_tether_obstacle){
            	count_tether_coll++;
				std::cout << " 		The agent TETHER in the state[" << i << "/"<< mt_.points.size() <<"] length["<< len_cat_<<"] position[" << j <<"/"<< points_catenary_.size() <<"] is in COLLISION ["
						<< dist_ <<" mts to obstacle/"<< distance_tether_obstacle<<"] pto["<< points_catenary_[j].x <<", "<< points_catenary_[j].y << ", "<< points_catenary_[j].z <<"] reel[" 
						<< p_reel_.x <<"," << p_reel_.y << "," << p_reel_.z <<"] UAV["<< p_uav_.x<<"," <<p_uav_.y <<"," << p_uav_.z << "]" <<std::endl; 
			}
		}
		if(count_tether_coll != aux_coll_theter_){
			v_pos_coll_tether.push_back(i);
			aux_coll_theter_ = count_tether_coll;
		}
	}

	if (count_ugv_coll > 0 || count_uav_coll > 0 || count_tether_coll > 0){
		ROS_INFO_COND(true, PRINTF_RED "CatenaryCheckerManager: Marsupial system in collision for RRT solution [ugv=%i  uav=%i  catenary=%i]",count_ugv_coll, count_uav_coll,count_tether_coll);
		ret_ = false;
	}
	else{
		ROS_INFO_COND(true, PRINTF_GREEN "CatenaryCheckerManager: Marsupial system collision free for RRT solution [ugv=%i  uav=%i  catenary=%i]",count_ugv_coll, count_uav_coll,count_tether_coll);
		ret_ = true;
	}

    std::cout << "	CatenaryCheckerManager finished" << std::endl << std::endl; 

	return ret_;
}

// bool CatenaryCheckerManager::CheckStatusCollision(vector<geometry_msgs::Vector3> v1_, vector<geometry_msgs::Quaternion> vq1_, vector<geometry_msgs::Vector3 >v2_, vector<tether_parameters> v3_)
// {
// 	bool ret_;

// 	geometry_msgs::Vector3 p_reel_; 
// 	std::vector<geometry_msgs::Vector3> points_tether_;
//     double len_cat_, dist_;
// 	count_ugv_coll = count_uav_coll = count_tether_coll = 0;

//     std::cout << std::endl << "	CatenaryCheckerManager started: analizing collision status for the marsupial agents" << std::endl; 
	
// 	double bound_coll_factor = 0.5;
// 	for (size_t i= 0 ; i <  v1_.size(); i++){

//         dist_ = getPointDistanceObstaclesMap(false, v1_[i],i,"UGV");
//         if (dist_ < distance_obstacle_ugv * bound_coll_factor){
// 			count_ugv_coll++;
//             std::cout << "      The agent UGV in the state = " << i << " is in COLLISION ["<< dist_ <<" mts to obstacle]" << std::endl; 
//         }

//         dist_ = getPointDistanceObstaclesMap(true, v2_[i],i,"UAV");
//         if (dist_ < distance_obstacle_uav * bound_coll_factor){
//             count_uav_coll++;
// 		    std::cout << "      The agent UAV in the state = " << i << " is in COLLISION ["<< dist_ <<" mts to obstacle]" << std::endl; 
// 		}
// 		p_reel_ = getReelNode(v1_[i],vq1_[i]);

// 		points_tether_.clear();
// 		GetTetherParameter GPP_;
// 		GPP_.getParabolaPoints(p_reel_, v2_[i], v3_[i], points_tether_);

// 		for (size_t j = 0 ; j < points_tether_.size() ; j++ ) {
//             dist_ = getPointDistanceObstaclesMap(true, points_tether_[j],i,"TETHER") ;
// 		    if( dist_ < (distance_tether_obstacle * bound_coll_factor)){
//             	count_tether_coll++;
//             	std::cout << " 		The agent TETHER in the state[" << i << "/"<< v1_.size()<<"] position[" << j <<"/"<< points_tether_.size() <<"] is in COLLISION ["
// 						<< dist_ <<" mts to obstacle/"<< distance_tether_obstacle*bound_coll_factor<<"] pto["<< points_tether_[j].x <<", "<< points_tether_[j].y << ", "<< points_tether_[j].z <<"] reel[" 
// 						<< p_reel_.x <<"," << p_reel_.y << "," << p_reel_.z <<"] UAV["<< v2_[i].x<<"," <<v2_[i].y <<"," <<v2_[i].z << "]" <<std::endl; 
// 			}
// 		}
// 	}

// 	if (count_ugv_coll > 0 || count_uav_coll > 0 || count_tether_coll > 0){
// 		ROS_INFO_COND(true, PRINTF_RED "\n \t\tcheckCollisionPathPlanner: Marsupial system in collision for solution [ugv=%i  uav=%i  catenary=%i]",count_ugv_coll, count_uav_coll,count_tether_coll);
// 		ret_ = false;
// 	}
// 	else{
// 		ROS_INFO_COND(true, PRINTF_GREEN "\n \t\tcheckCollisionPathPlanner: Marsupial system collision free for solution [ugv=%i  uav=%i  catenary=%i]",count_ugv_coll, count_uav_coll,count_tether_coll);
// 		ret_ = true;
// 	}

//     std::cout << "	CatenaryCheckerManager finished" << std::endl << std::endl; 

// 	return ret_;
// }

bool CatenaryCheckerManager::CheckStatusTetherCollision(vector<geometry_msgs::Vector3> v1_, vector<geometry_msgs::Quaternion> vq1_, vector<geometry_msgs::Vector3 >v2_, vector<tether_parameters> v3_, vector<float> length_)
{
	bool ret_;

	geometry_msgs::Vector3 p_reel_; 
	std::vector<geometry_msgs::Vector3> points_tether_;
    double len_cat_, dist_;
	int first_coll_, last_coll_, count_total_tether_coll_;
	count_ugv_coll = count_uav_coll = count_tether_coll = count_total_tether_coll_ = 0;

    std::cout << std::endl << "	CatenaryCheckerManager started: analizing collision status for the marsupial agents" << std::endl; 
	
	double bound_coll_factor = 0.5;
	for (size_t i= 0 ; i <  v1_.size(); i++){

        dist_ = getPointDistanceObstaclesMap(false, v1_[i],i,"UGV");
        if (dist_ < distance_obstacle_ugv * bound_coll_factor){
			count_ugv_coll++;
            std::cout << "      The agent UGV in the state = " << i << " is in COLLISION ["<< dist_ <<" mts to obstacle]" << std::endl; 
        }

        dist_ = getPointDistanceObstaclesMap(true, v2_[i],i,"UAV");
        if (dist_ < distance_obstacle_uav * bound_coll_factor){
            count_uav_coll++;
		    std::cout << "      The agent UAV in the state = " << i << " is in COLLISION ["<< dist_ <<" mts to obstacle]" << std::endl; 
		}
		p_reel_ = getReelNode(v1_[i],vq1_[i]);

		points_tether_.clear();
		GetTetherParameter GPP_;
		if (use_catenary_as_tether)
			GPP_.getCatenaryPoints(p_reel_, v2_[i], v3_[i], points_tether_, length_[i]);
		else
			GPP_.getParabolaPoints(p_reel_, v2_[i], v3_[i], points_tether_);

		count_tether_coll = 0;
		first_coll_ = last_coll_ = -1;
		for (size_t j = 0 ; j < points_tether_.size() ; j++ ) {
            dist_ = getPointDistanceObstaclesMap(true, points_tether_[j],i,"TETHER") ;
		    // if( dist_ < (distance_tether_obstacle * bound_coll_factor)){
		    // if( dist_ < (0.02)){
		    if( dist_ < (distance_tether_obstacle)){
            	count_tether_coll++;
				if (first_coll_ == -1)
					first_coll_ = j;
				last_coll_ = j;
            	// std::cout << " 		The agent TETHER in the state[" << i << "/"<< v1_.size()<<"] position[" << j <<"/"<< points_tether_.size() <<"] is in COLLISION ["
				// << dist_ <<" mts to obstacle/"<< distance_tether_obstacle*bound_coll_factor<<"] pto["<< points_tether_[j].x <<", "<< points_tether_[j].y << ", "<< points_tether_[j].z <<"] reel[" 
				// << p_reel_.x <<"," << p_reel_.y << "," << p_reel_.z <<"] UAV["<< v2_[i].x<<"," <<v2_[i].y <<"," <<v2_[i].z << "]" <<std::endl; 
			}
		}
		if (count_tether_coll>0 )
			std::cout << "TETHER ["<< i <<"] in collision. Total Points:["<< count_tether_coll <<"] , between["<< first_coll_ <<"-"<<last_coll_ <<"]" << std::endl;
		count_total_tether_coll_ = count_total_tether_coll_ + count_tether_coll;
	}

	if (count_ugv_coll > 0 || count_uav_coll > 0 || count_total_tether_coll_ > 0){
		ROS_INFO_COND(true, PRINTF_RED "\n \t\tcheckCollisionPathPlanner: Marsupial system in collision for solution [ugv=%i  uav=%i  tether=%i]",count_ugv_coll, count_uav_coll,count_total_tether_coll_);
		ret_ = false;
	}
	else{
		ROS_INFO_COND(true, PRINTF_GREEN "\n \t\tcheckCollisionPathPlanner: Marsupial system collision free for solution [ugv=%i  uav=%i  tether=%i]",count_ugv_coll, count_uav_coll,count_total_tether_coll_);
		ret_ = true;
	}

    std::cout << "	CatenaryCheckerManager finished" << std::endl << std::endl; 

	return ret_;
}

bool CatenaryCheckerManager::CheckFreeCollisionPoint(geometry_msgs::Vector3 p_, string mode_, int pose_)
{
	double sefaty_distance_;
	bool use_distance_function_;
	if(mode_ == "UGV"){
		sefaty_distance_ = distance_obstacle_ugv;
		use_distance_function_ = false;
	}else if(mode_ == "UAV"){
		sefaty_distance_ = distance_obstacle_uav;
		use_distance_function_ = true;
	}else{
		sefaty_distance_ = distance_tether_obstacle;
		use_distance_function_ = true;
	}	    
    
	ROS_ERROR("CatenaryCheckerManager::CheckFreeCollisionPoint : p[%f %f %f] pose_[%i] mode=%s",p_.x,p_.y,p_.z, pose_, mode_.c_str());
	double dist_ = getPointDistanceObstaclesMap(use_distance_function_, p_, pose_ ,mode_) ;
	
	if( dist_ < sefaty_distance_){
		ROS_ERROR("CatenaryCheckerManager::CheckFreeCollisionPoint : %s use_method[%s] Point in collision p[%.3f %.3f %.3f] d[%.3f/%.3f]",mode_.c_str(), use_distance_function_?"true":"false",p_.x,p_.y,p_.z, dist_, sefaty_distance_);
		return false;
	}else{
		// ROS_INFO("CatenaryCheckerManager::CheckFreeCollisionPoint : Point Not collision p[%.3f %.3f %.3%] d[%.3%/%.3f]",p_.x,p_.y,p_.z,dist_, sefaty_distance_);
		return true;
	}
}

geometry_msgs::Vector3 CatenaryCheckerManager::getReelNode(const geometry_msgs::Vector3 p_, const geometry_msgs::Quaternion q_)
{
	geometry_msgs::Vector3 pos_reel;
	double yaw_ugv;

	yaw_ugv = getYawFromQuaternion(q_.x, q_.y, q_.z, q_.w);
	double lengt_vec =  sqrt(p_reel_ugv.x*p_reel_ugv.x + p_reel_ugv.y*p_reel_ugv.y);
	pos_reel.x = p_.x + lengt_vec *cos(yaw_ugv); 
	pos_reel.y = p_.y + lengt_vec *sin(yaw_ugv);
	pos_reel.z = p_.z + p_reel_ugv.z ; 
	return pos_reel;
}

double CatenaryCheckerManager::getYawFromQuaternion(double x_, double y_, double z_, double w_)
{
	double roll_, pitch_, yaw_;

	tf::Quaternion q( x_, y_, z_, w_);
	tf::Matrix3x3 M(q);	
	M.getRPY(roll_, pitch_, yaw_);

	return yaw_;
}