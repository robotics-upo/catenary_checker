#include <catenary_checker/catenary_checker_node.h>

catenaryChecker::catenaryChecker(ros::NodeHandlePtr nh)
{
  tf2_ros::TransformListener tfl(tf_buffer);

  // It assumes that nh is a local ros node (~)
  base_frame = nh->param<std::string>("base_frame_id", std::string("base_link"));
  global_frame = nh->param<std::string>("global_frame_id", std::string("map"));
  plane_dist = nh->param<float>("plane_dist", 0.5f);
  publish_pc = nh->param<bool>("publish_pc", true);
  publish_marker = nh->param<bool>("publish_marker", true);
     
  // DBScan params
  dbscan_min_points = nh->param<int>("dbscan_min_points", 10);
  dbscan_epsilon = nh->param<float>("dbscan_epsilon", 0.1);
  dbscan_gamma = nh->param<float>("dbscan_gamma", 0.1);
  dbscan_theta = nh->param<float>("dbscan_theta", 0.1);
  use_dbscan_lines = nh->param<bool>("use_dbscan_lines", false);

  precomputed_file = nh->param<std::string>("precomputed_file",
                                            static_cast<std::string>(""));
                                            
  bool use_parabola = false;
  nh->param("use_parabola", use_parabola, (bool)false);

  if (precomputed_file != "" && use_parabola) {
    precomputePlanes();
  } 

  ROS_INFO("Catenary checker node. Global frame: %s. Base frame: %s", base_frame.c_str(), global_frame.c_str());
  ROS_INFO("Plane dist: %f. Publish pc, marker: %d, %d ", plane_dist, publish_pc, publish_marker);
  ROS_INFO("DBscan epsilon: %f. DBScan min points: %d ", dbscan_epsilon, dbscan_min_points);
  if (use_dbscan_lines) {
    ROS_INFO("Using DBScan Lines. Gamma: %f. Theta: %f", dbscan_gamma, dbscan_theta);
  }

  if (publish_pc) {
    pc_publisher = nh->advertise<sensor_msgs::PointCloud2>("plane_pc", 2, true);
  }

  if (publish_marker) {
    marker_publisher = nh->advertise<visualization_msgs::Marker>("segmented_pc", 2, true);
  }
}

void catenaryChecker::getPointCloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
{
  pc = pc_msg;
  ROS_INFO(PRINTF_YELLOW "catenaryChecker::getPointCloud: Received Point Cloud heigth=%i width=%i",pc->height, pc->width);
}

//! Gets a point and checks if there exists
bool catenaryChecker::analyticalCheckCatenary(const geometry_msgs::Point &pi_, const geometry_msgs::Point &pf_, std::vector<geometry_msgs::Point> &pts_c_)
{
  static int seq = 0;
  bool ret_val = false;

  pcl::PointXYZ robot(static_cast<float>(pi_.x),
                      static_cast<float>(pi_.y),
                      static_cast<float>(pi_.z));
  pcl::PointXYZ target(static_cast<float>(pf_.x),
                       static_cast<float>(pf_.y),
                       static_cast<float>(pf_.z));
  geometry_msgs::Point pts_; // To save Catenary point
  if ( (fabs(robot.x - target.x) < 0.01) && (fabs(robot.y == target.y) < 0.01 ) )
  {
    int num_pts_per_unit_length = 10;
    length_cat = fabs(robot.z - target.z) *1.01;
    // std::cout << "catenaryChecker::getPointCloud -if- length_cat= " << length_cat << std::endl;
    int num_pts_cat_ = round( (double)num_pts_per_unit_length * length_cat);
    for (size_t i = 0 ; i < num_pts_cat_ ; i ++){
      pts_.x = robot.x; 
      pts_.y = robot.y; 
      pts_.z = robot.z + (length_cat/num_pts_cat_)*(i+1); 
      pts_c_.push_back(pts_);
        // double dist_cat_obs = getPointDistanceFullMap(use_distance_function, pts_);
        // if (d_min_point_cat > dist_cat_obs){
        //   min_dist_obs_cat = dist_cat_obs;
        //   d_min_point_cat = dist_cat_obs;
        // }
    }
    min_dist_obs_cat = -1.0;
    // std::cout << "Parabola en 1 plano Calculada" << std::endl;
    get_catenary = false;
    return false;
  }


  // If there is a filename --> we use precomputation for planes
  if (precomputed_file != "") {
    ret_val = precomputedCheckCatenary(robot, target, pts_c_);
  } else {
    // If not, we compute the obstacles in the plane
    // and then check for Parabola
    
    
    DBSCAN *dbscan = NULL;

    pcl::PointCloud<pcl::PointXYZ> pcl_pc;
    pcl::PCLPointCloud2 pcl_pc2;

    pcl_conversions::toPCL(*pc,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_pc); // TODO: Avoid conversions!!
      
    std::cout << "catenaryChecker::getPointCloud: Robot:[" << robot.x << "," << robot.y << "," << robot.z << "] , Target:[" << target.x << "," << target.y << "," << target.z << "]" << std::endl;
    std::cout << "Preparando para calcular Plano 2D" << std::endl;

    auto points_2d = project2D(pcl_pc, robot, target, plane_dist);
    // ROS_INFO("Obtained 2D cloud projection. Number of points: %lu", points_2d.size());
    if (publish_pc) {
      std::cout << "Preparando para calcular Plano 3D" << std::endl;
      auto points_3d = reproject3D(points_2d, robot, target);
      pcl::toPCLPointCloud2(points_3d, pcl_pc2);
          
      sensor_msgs::PointCloud2 out_pc2;
      pcl_conversions::moveFromPCL(pcl_pc2, out_pc2);
      out_pc2.header.stamp = ros::Time::now();
      out_pc2.header.seq = seq++;
      out_pc2.header.frame_id = global_frame;

      pc_publisher.publish(out_pc2);
      std::cout << "Plano 3D calculado" << std::endl;
    }
    std::cout << "Preparando CLUSTERIZE" << std::endl;
    if (use_dbscan_lines) {
      dbscan = clusterize_lines(points_2d, dbscan_min_points, dbscan_epsilon, dbscan_gamma, dbscan_theta);
    } else {
      dbscan = clusterize(points_2d, dbscan_min_points, dbscan_epsilon);
    }
    std::cout << "Hecho el CLUSTERIZE" << std::endl;

    // ROS_INFO("Clusterized with DBSCAN. N_clusters: %d. \tN_points: %lu",dbscan->getNClusters(), dbscan->getPoints().size());
    if (publish_marker) {
      marker_publisher.publish(pointsToMarker(dbscan->getPoints(), global_frame, 1000));
    }

    int num_pts_per_unit_length = 10;
      
    pts_c_.clear();
      
    if ( (fabs(robot.x - target.x) < 0.01) && (fabs(robot.y == target.y) < 0.01 ) )
      {
        length_cat = fabs(robot.z - target.z) *1.01;
        // std::cout << "catenaryChecker::getPointCloud -if- length_cat= " << length_cat << std::endl;
        int num_pts_cat_ = round( (double)num_pts_per_unit_length * length_cat);
        for (size_t i = 0 ; i < num_pts_cat_ ; i ++){
          pts_.x = robot.x; 
          pts_.y = robot.y; 
          pts_.z = robot.z + (length_cat/num_pts_cat_)*(i+1); 
          pts_c_.push_back(pts_);
          // double dist_cat_obs = getPointDistanceFullMap(use_distance_function, pts_);
          // if (d_min_point_cat > dist_cat_obs){
          //   min_dist_obs_cat = dist_cat_obs;
          //   d_min_point_cat = dist_cat_obs;
          // }
        }
        min_dist_obs_cat = -1.0;
        std::cout << "Parabola en 1 plano Calculada" << std::endl;
      }
    else
      {
        std::cout << "Compute Obstacles using DBSCAN" << std::endl;
        //Tranlate to Obstacles 2D
        Scenario scenario = getObstacles(dbscan, robot, target); 
    // std::cout << "Compute getVerticalPlane" << std::endl;
        auto plane = getVerticalPlane(robot,target); 
        Point2D A(robot.y * plane.a - robot.x * plane.b, robot.z);
        Point2D B(target.y * plane.a - target.x * plane.b, target.z);
  // std::cout << "catenaryChecker::getPointCloud: A:[" << A.x << "," << A.y << "] , B:[" << B.x << ","<< B.y <<"]" << std::endl;
        Parabola parabola;
        ret_val = parabola.approximateParabola(scenario, A, B);
        if (ret_val) {
        // Represent parabola and get min distance to obstacles
            // ROS_INFO("Got parabola: %s", parabola.toString().c_str()); 
          pcl::PointCloud<pcl::PointXYZ> pc_catenary;
          length_cat = getParabolaPoints(parabola, robot, target, pc_catenary);

          for (size_t i = 0 ; i < pc_catenary.size() ; i ++){
            // if (i%quot == 0){
            pts_.x = pc_catenary.points[pc_catenary.size()-(1+i)].x; 
            pts_.y = pc_catenary.points[pc_catenary.size()-(1+i)].y; 
            pts_.z = pc_catenary.points[pc_catenary.size()-(1+i)].z; 
            pts_c_.push_back(pts_);
          }
          min_dist_obs_cat = -1.0;
        }
        
      }
    delete dbscan;
  }
  
  get_catenary = ret_val;
  return ret_val;
}

bool catenaryChecker::precomputePlanes() {
  struct timespec start_planes, finish_planes;
  ROS_INFO("Precomputing planes:");
  clock_gettime(CLOCK_REALTIME, &start_planes);
  ps.reset(new PreprocessedScenario(precomputed_file));

  if (ps->size() == 0) {
    ROS_INFO("File %s not found. Precomputing.", precomputed_file.c_str());
    ps->_min.x = grid_3D->min_X; 
    ps->_min.y = grid_3D->min_Y;
    ps->_min.x = grid_3D->max_X;
    ps->_max.y = grid_3D->max_Y;   
    ps->_max_z = grid_3D->max_Z;
                                
    ps->PC_Callback(pc);
  } else {
    ROS_INFO("Obtaining the precomputed scenario from file.");
  }

  ROS_INFO("Number of precomputed directions: %d", static_cast<int>(ps->size()));
  clock_gettime(CLOCK_REALTIME, &finish_planes);
  auto sec_planes = finish_planes.tv_sec - start_planes.tv_sec;
  auto msec_planes = finish_planes.tv_nsec - start_planes.tv_nsec;
  precomputing_time = (msec_planes + sec_planes * 1000000000.0)/1000000000.0;
  std::cout << "Precomputed time: " << precomputing_time << std::endl << std::endl;

  return true;
}

bool catenaryChecker::precomputedCheckCatenary(const pcl::PointXYZ &pi_,
                                               const pcl::PointXYZ &pf_,
                                               std::vector<geometry_msgs::Point> &pts_c_)
{
  if (ps == NULL) {
    precomputePlanes();
  }

  get_catenary = ps->checkCatenary(pi_, pf_, pts_c_) > -0.5f;

  return get_catenary;

  // pcl::PointCloud<pcl::PointXYZ> pc_catenary;
  // length_cat = getParabolaPoints(ps->_parabola, pi_, pf_, pc_catenary);

  // geometry_msgs::Point pts_; // To save Catenary point
  // pts_c_.clear();
  // for (size_t i = 0 ; i < pc_catenary.size() ; i ++){
  //   pts_.x = pc_catenary.points[pc_catenary.size()-(1+i)].x; 
  //   pts_.y = pc_catenary.points[pc_catenary.size()-(1+i)].y; 
  //   pts_.z = pc_catenary.points[pc_catenary.size()-(1+i)].z; 
  //   pts_c_.push_back(pts_);
  // }
}

std_msgs::ColorRGBA catenaryChecker::getColor(int num) {
  
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

std_msgs::ColorRGBA catenaryChecker::getGray(int num) {
  int i = num % 5;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.b = color.r = color.g = i*0.15 + 0.2;

  if (num < 0)
    color.r = color.b = color.g = 1.0;

  return color;
}

visualization_msgs::Marker catenaryChecker::pointsToMarker(const std::vector<Point> &points, const std::string frame_id, int n_lines){
  visualization_msgs::Marker _marker;
  _marker.header.frame_id = frame_id;
  _marker.header.stamp = ros::Time::now();
  _marker.ns = "segmented_pc";
  _marker.id = 0;
  _marker.type = visualization_msgs::Marker::POINTS;
  _marker.action = visualization_msgs::Marker::ADD;
  _marker.pose.position.x = 0;
  _marker.pose.position.y = 0;
  _marker.pose.position.z = 0;
  _marker.pose.orientation.x = 0.0;
  _marker.pose.orientation.y = 0.0;
  _marker.pose.orientation.z = 0.0;
  _marker.pose.orientation.w = 1.0;
  _marker.scale.x = 0.15;
  _marker.scale.y = 0.15;
  _marker.scale.z = 0.15;
  _marker.color.a = 1.0; // Don't forget to set the alpha!
  _marker.color.r = 0.0;
  _marker.color.g = 1.0;
  _marker.color.b = 1.0;

  for (auto p:points) {
    if (fabs(p.x) > 100.0 || fabs(p.y) > 100.0)
      continue;

    geometry_msgs::Point gp;
            
    gp.x = p.x;
    gp.y = p.y;
    gp.z = p.z;
    _marker.points.push_back(gp);
    if(p.clusterID < n_lines)
      _marker.colors.push_back(getColor(p.clusterID));
    else
      _marker.colors.push_back(getGray(p.clusterID));
  }

  return _marker;
}

double catenaryChecker::getPointDistanceFullMap(bool use_dist_func_, geometry_msgs::Point p_)
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

void catenaryChecker::getDataForDistanceinformation(Grid3d *grid3D_,
                                                    const sensor_msgs::PointCloud2::ConstPtr& msg,
                                                    bool use_distance_function_)
{
	grid_3D = grid3D_;
  nn_obs.setInput(*msg);
  use_distance_function = use_distance_function_;
}

