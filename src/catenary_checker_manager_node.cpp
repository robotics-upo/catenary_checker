#include <catenary_checker/catenary_checker_manager.h>

int main(int argc, char **argv)
{
  std::string node_name = "catenary_checker_manager_node";

	ros::init(argc, argv, node_name);

  ros::NodeHandle nh("~");

  std::cout << "Initializing node: " << node_name << std::endl;
  CatenaryCheckerManager checker(node_name);

  double distance_catenary_obstacle, distance_obstacle_ugv, distance_obstacle_uav;
  double length_tether_max;
  geometry_msgs::Point min, max;

  bool use_parabola, use_both;
  bool use_distance_function = true;
  double map_resolution = 0.05;

  Grid3d g(node_name);

  nh.param("ws_x_max", max.x, (double)30);
  nh.param("ws_y_max", max.y, (double)30);
  nh.param("ws_z_max", max.z, (double)30);
  nh.param("ws_x_min", min.x, (double)0);
  nh.param("ws_y_min", min.y, (double)0);
  nh.param("ws_z_min", min.z, (double)0);

  nh.param("map_resolution", map_resolution, (double)0.05);
  nh.param("distance_obstacle_ugv", distance_obstacle_ugv, (double)1.0);
  nh.param("distance_obstacle_uav", distance_obstacle_uav, (double)1.0);
  nh.param("distance_catenary_obstacle", distance_catenary_obstacle, (double)0.1);
  nh.param("length_tether_max", length_tether_max, (double)10.0);

  nh.param("use_parabola", use_parabola, (bool)true);
  nh.param("use_both", use_both, (bool)true);

  geometry_msgs::Point pos_r;
  nh.param("pos_reel_x", pos_r.x);
  nh.param("pos_reel_y", pos_r.y);
  nh.param("pos_reel_z", pos_r.z);

  checker.init(&g, distance_catenary_obstacle, distance_obstacle_ugv, distance_obstacle_uav,
	                length_tether_max, min.z, map_resolution, use_parabola, use_distance_function, pos_r,
			            false, !use_parabola, use_both);

	ros::Rate loop_rate(5);

    for(int i = 0; i < 10 && ros::ok() ; i++){

        ros::spinOnce();

        loop_rate.sleep();
    }

    while (ros::ok()) {
      geometry_msgs::Point A, B;
      std::vector<geometry_msgs::Point> points;
      cout << "Please enter A: ";
      cin >> A.x >> A.y >> A.z;
      cout << "Please enter B: ";
      cin >> B.x >> B.y >> B.z;
      
      if (checker.searchCatenary(A, B, points)) {
        cout << "´There exists a catenary" << endl;
      } else {
        cout << "´There not exists a catenary" << endl;
        
      }
      sleep(1);
      ros::spinOnce();
    }

    return 0;
}
 
