#include <catenary_checker/check_collision_path_planner.h>


checkCollisionPathPlanner::checkCollisionPathPlanner(std::string node_name_, Grid3d *grid_3D_, geometry_msgs::Vector3 p_reel_ugv_, 
													double d_obs_ugv_, double d_obs_uav_, double d_obs_tether_,
													double length_tether_max_, double ws_z_min_, double step_, bool use_parable_, bool use_distance_function_)
{
    //The tf buffer is used to lookup the base link position(tf from world frame to robot base frame)
    node_name = node_name_;
	grid_3D = grid_3D_;
	pc_obs_ugv = pc_;
	// nn_obs_ugv.setInput(*pc_obs_ugv);
	p_reel_ugv = p_reel_ugv_;
	nh.reset(new ros::NodeHandle("~"));

	distance_obstacle_ugv = d_obs_ugv_;
	distance_obstacle_uav = d_obs_uav_;
	distance_tether_obstacle = d_obs_tether_;

    // CheckCM->Init(grid_3D, distance_tether_obstacle, length_tether_max, ws_z_min, map_resolution, use_parable, use_distance_function);
	
}

bool checkCollisionPathPlanner::CheckStatus(trajectory_msgs::MultiDOFJointTrajectory mt_, std::vector<double> ct_)
{
	bool ret_;

	geometry_msgs::Vector3 p_ugv_, p_uav_, p_reel_; 
	std::vector<geometry_msgs::Vector3> points_catenary_;
    double len_cat_, dist_;
	bisectionCatenary bc;
	count_ugv_coll = count_uav_coll = count_tether_coll = 0;
	v_pos_coll_tether.clear();

    std::cout << std::endl << "	checkCollisionPathPlanner started: analizing collision status for the marsupial agents" << std::endl; 
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
		

		dist_ = CheckCM->getPointDistanceObstaclesMap(false, p_ugv_,i,"UGV");
        if (dist_ < distance_obstacle_ugv){
			count_ugv_coll++;
            std::cout << "      The agent UGV in the state = " << i << " is in COLLISION ["<< dist_ <<" mts to obstacle]" << std::endl; 
        }

        dist_ = CheckCM->getPointDistanceObstaclesMap(true, p_uav_,i,"UAV");
        if (dist_ < distance_obstacle_uav){
            count_uav_coll++;
		    std::cout << "      The agent UAV in the state = " << i << " idistance_tether_obstacles in COLLISION ["<< dist_ <<" mts to obstacle]" << std::endl; 
		}

		p_reel_ = getReelNode(p_,q_);

		points_catenary_.clear();
		bool just_one_axe = bc.configBisection(len_cat_, p_reel_.x, p_reel_.y, p_reel_.z, p_uav_.x, p_uav_.y, p_uav_.z);
		bc.getPointCatenary3D(points_catenary_, false);
		for (size_t j = 0 ; j < points_catenary_.size() ; j++ ) {
            dist_ = CheckCM->getPointDistanceObstaclesMap(true, points_catenary_[j],i,"TETHER") ;
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
		ROS_INFO_COND(true, PRINTF_RED "checkCollisionPathPlanner: Marsupial system in collision for RRT solution [ugv=%i  uav=%i  catenary=%i]",count_ugv_coll, count_uav_coll,count_tether_coll);
		ret_ = false;
	}
	else{
		ROS_INFO_COND(true, PRINTF_GREEN "checkCollisionPathPlanner: Marsupial system collision free for RRT solution [ugv=%i  uav=%i  catenary=%i]",count_ugv_coll, count_uav_coll,count_tether_coll);
		ret_ = true;
	}

    std::cout << "	checkCollisionPathPlanner finished" << std::endl << std::endl; 

	return ret_;
}

bool checkCollisionPathPlanner::CheckStatus(vector<geometry_msgs::Vector3> v1_, vector<geometry_msgs::Quaternion> vq1_, vector<geometry_msgs::Vector3 >v2_, vector<parable_parameters> v3_)
{
	bool ret_;

	geometry_msgs::Vector3 p_reel_; 
	std::vector<geometry_msgs::Vector3> points_parable_;
    double len_cat_, dist_;
	count_ugv_coll = count_uav_coll = count_tether_coll = 0;

	// std::cout << "v1_= " << v1_.size() << " vq1_=" << vq1_.size() << " v2_=" << v2_.size() << " v3_=" << v3_.size() << std::endl;
    std::cout << std::endl << "	checkCollisionPathPlanner started: analizing collision status for the marsupial agents" << std::endl; 
	
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

		points_parable_.clear();
		GetParabolaParameter GPP_;
		GPP_.getParabolaPoints(p_reel_, v2_[i], v3_[i], points_parable_);
		// std::cout << "points_parable_.size() = " << points_parable_.size() << std::endl;

		for (size_t j = 0 ; j < points_parable_.size() ; j++ ) {
            dist_ = getPointDistanceObstaclesMap(true, points_parable_[j],i,"TETHER") ;
		    if( dist_ < (distance_tether_obstacle * bound_coll_factor)){
            	count_tether_coll++;
            	std::cout << " 		The agent TETHER in the state[" << i << "/"<< v1_.size()<<"] position[" << j <<"/"<< points_parable_.size() <<"] is in COLLISION ["
						<< dist_ <<" mts to obstacle/"<< distance_tether_obstacle*bound_coll_factor<<"] pto["<< points_parable_[j].x <<", "<< points_parable_[j].y << ", "<< points_parable_[j].z <<"] reel[" 
						<< p_reel_.x <<"," << p_reel_.y << "," << p_reel_.z <<"] UAV["<< v2_[i].x<<"," <<v2_[i].y <<"," <<v2_[i].z << "]" <<std::endl; 
			}
		}
	}

	if (count_ugv_coll > 0 || count_uav_coll > 0 || count_tether_coll > 0){
		ROS_INFO_COND(true, PRINTF_RED "\n \t\tcheckCollisionPathPlanner: Marsupial system in collision for solution [ugv=%i  uav=%i  catenary=%i]",count_ugv_coll, count_uav_coll,count_tether_coll);
		ret_ = false;
	}
	else{
		ROS_INFO_COND(true, PRINTF_GREEN "\n \t\tcheckCollisionPathPlanner: Marsupial system collision free for solution [ugv=%i  uav=%i  catenary=%i]",count_ugv_coll, count_uav_coll,count_tether_coll);
		ret_ = true;
	}

    std::cout << "	checkCollisionPathPlanner finished" << std::endl << std::endl; 

	return ret_;
}

bool checkCollisionPathPlanner::CheckFreeCollisionPoint(geometry_msgs::Vector3 p_, string mode_, int pose_)
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
    
	double dist_ = getPointDistanceObstaclesMap(use_distance_function_, p_, pose_ ,mode_) ;
	
	if( dist_ < sefaty_distance_){
		ROS_ERROR("checkCollisionPathPlanner::CheckFreeCollisionPoint : %s use_method[%s] Point in collision p[%.3f %.3f %.3f] d[%.3f/%.3f]",mode_.c_str(), use_distance_function_?"true":"false",p_.x,p_.y,p_.z, dist_, sefaty_distance_);
		return false;
	}else{
		// ROS_INFO("checkCollisionPathPlanner::CheckFreeCollisionPoint : Point Not collision p[%.3f %.3f %.3%] d[%.3%/%.3f]",p_.x,p_.y,p_.z,dist_, sefaty_distance_);
		return true;
	}
}

geometry_msgs::Vector3 checkCollisionPathPlanner::getReelNode(const geometry_msgs::Vector3 p_, const geometry_msgs::Quaternion q_)
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

double checkCollisionPathPlanner::getYawFromQuaternion(double x_, double y_, double z_, double w_)
{
	double roll_, pitch_, yaw_;

	tf::Quaternion q( x_, y_, z_, w_);
	tf::Matrix3x3 M(q);	
	M.getRPY(roll_, pitch_, yaw_);

	return yaw_;
}

// double checkCollisionPathPlanner::getPointDistanceObstaclesMap(bool ugv_obstacle_, geometry_msgs::Vector3 p_, int pose_, string msg_)
// {
// 	double dist;

// 	if(!ugv_obstacle_){
// 		bool is_into_ = grid_3D->isIntoMap(p_.x,p_.y,p_.z);
// 		if(is_into_)
// 			dist =  grid_3D->getPointDist((double)p_.x,(double)p_.y,(double)p_.z) ;
// 		else{
//             std::cout << "\tThe agent " << msg_ << " in the state = " << pose_ << " is out of the GRID["<<p_.x<< ", " << p_.y << ", " <<p_.z << "]"<< std::endl; 
// 			dist = -1.0;
//         }
//     }
// 	else{
// 		Eigen::Vector3d pos_, obs_;
// 		pos_.x() = p_.x;
// 		pos_.y() = p_.y; 
// 		pos_.z() = p_.z; 
// 		obs_= nn_obs_ugv.nearestObstacleMarsupial(nn_obs_ugv.kdtree, pos_, nn_obs_ugv.obs_points);

// 		if (pos_.z() < obs_.z()+0.1) // - 0.05 is because UGV can drive above 5 cm obstacles
// 			dist = sqrt(pow(obs_.x()-pos_.x(),2) + pow(obs_.y()-pos_.y(),2) + pow(obs_.z()-pos_.z(),2));
// 		else	
// 			dist = 1.0; //For whole obstacle under or same altitud than UGV
// 	}

// 	return dist;
// }