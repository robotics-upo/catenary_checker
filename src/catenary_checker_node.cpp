#include <vector>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <catenary_checker/catenary_checker.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <catenary_checker/obstacle_2d.hpp>
#include <catenary_checker/parable.hpp>

//! Global data
sensor_msgs::PointCloud2 pc;
tf2_ros::Buffer tf_buffer;
std::string base_frame, global_frame;
float plane_dist = 0.5;
// DBScan related
int dbscan_min_points;
float dbscan_epsilon, dbscan_gamma, dbscan_theta;
bool use_dbscan_lines;

ros::Publisher pc_publisher, marker_publisher;
bool publish_pc = true;
bool publish_marker = true;

visualization_msgs::Marker pointsToMarker(const std::vector<Point> &points,
					  const std::string frame_id, int n_lines = -1);

void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{
  pc = *pc_msg;
}

Obstacle2D toObstacle(const std::vector<Point> &obs);

//! Gets a point and checks if there exists
void checkCatenary(const geometry_msgs::PoseStampedConstPtr &target_pose)
{
  static int seq = 0;
  // Get the pose of the robot or die
  geometry_msgs::TransformStamped transformStamped;
  DBSCAN *dbscan = NULL;
  try{
    // get the location of the robot
    transformStamped = tf_buffer.lookupTransform(global_frame, base_frame,
						 ros::Time(0));
    pcl::PointXYZ robot(static_cast<float>(transformStamped.transform.translation.x), 
			static_cast<float>(transformStamped.transform.translation.y),
			static_cast<float>(transformStamped.transform.translation.z));
         
    pcl::PointXYZ target(static_cast<float>(target_pose->pose.position.x),
			 static_cast<float>(target_pose->pose.position.y),
			 static_cast<float>(target_pose->pose.position.z));
    pcl::PointCloud<pcl::PointXYZ> pcl_pc;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::moveToPCL(pc, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_pc); // TODO: Avoid conversions!!

    ROS_INFO("Robot: %f %f %f \t Target: %f %f %f", robot.x, robot.y, robot.z,
	     target.x, target.y, target.z);
    ROS_INFO("Got PC. Sizes: original:%lu \t pcl_2:%lu \t pcl_pc:%lu",
	     pc.data.size(), pcl_pc2.data.size(), pcl_pc.size());

    auto points_2d = project2D(pcl_pc, robot, target, plane_dist);
    ROS_INFO("Obtained 2D cloud projection. Number of points: %lu", points_2d.size());
    if (publish_pc) {
      auto points_3d = reproject3D(points_2d, robot, target);

      ROS_INFO("Publishing 2D cloud reconverted to 3D . Number of points: %lu",
	       points_3d.size());

      pcl::toPCLPointCloud2(points_3d, pcl_pc2);
         
      sensor_msgs::PointCloud2 out_pc2;
      pcl_conversions::moveFromPCL(pcl_pc2, out_pc2);
      out_pc2.header.stamp = ros::Time::now();
      out_pc2.header.seq = seq++;
      out_pc2.header.frame_id = global_frame;

      pc_publisher.publish(out_pc2);
    }
         
    if (use_dbscan_lines) {
      dbscan = clusterize_lines(points_2d, dbscan_min_points, dbscan_epsilon,
				dbscan_gamma, dbscan_theta);
    } else {
      dbscan = clusterize(points_2d, dbscan_min_points, dbscan_epsilon);
    }
    ROS_INFO("Clusterized with DBSCAN. N_clusters: %d. \tN_points: %lu",
	     dbscan->getNClusters(), dbscan->getPoints().size());
    if (publish_marker) {
      ROS_INFO("Trying to publish marker");
      marker_publisher.publish(pointsToMarker(dbscan->getPoints(), global_frame, 1000));
    }
    ROS_INFO("Marker published");

    //Tranlate to Obstacles 2D
    std::vector<Obstacle2D> scenario;
    for (int i = 1; i < dbscan->getNClusters(); i++) {
      auto cluster = dbscan->getCluster(i);
      ROS_INFO("Cluster %d. Size: %lu", i, cluster.size());
      if (cluster.size() > dbscan_min_points) {
	auto curr_obstacle = toObstacle(cluster);
	scenario.push_back(curr_obstacle);
      }
    }
    ROS_INFO("Got the scenario");
    // Get the initial parable (line between A and B)
    auto plane = getVerticalPlane(robot,target);
    Point2D A(robot.y * plane.a - robot.x * plane.b, robot.z);
    Point2D B(target.y * plane.a - target.x * plane.b, target.z);
    Parable parable;
    parable.approximateParable(scenario, A, B);

    ROS_INFO("Got parable: %s", parable.toString().c_str()); 
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  delete dbscan;
}

int main (int argc, char **argv)
{
  // --- Inicializacion de ROS. No hace falta tocar
  std::string node_name = "catenary_checker";
  ros::init(argc, argv, node_name);

  ros::NodeHandle nh, lnh("~");

  tf2_ros::TransformListener tfl(tf_buffer);

  // Subscribe the node to the point cloud from the ROS bag file.
  // The topic has to be remapped to points2
  ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>("points2", 1, pointCloudCb);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("pose", 1, checkCatenary);

  base_frame = lnh.param<std::string>("base_frame_id", std::string("base_link"));
  global_frame = lnh.param<std::string>("global_frame_id", std::string("map"));
  plane_dist = lnh.param<float>("plane_dist", 0.5f);
  publish_pc = lnh.param<bool>("publish_pc", true);
  publish_marker = lnh.param<bool>("publish_marker", true);
     
  // DBScan params
  dbscan_min_points = lnh.param<int>("dbscan_min_points", 10);
  dbscan_epsilon = lnh.param<float>("dbscan_epsilon", 0.1);
  dbscan_gamma = lnh.param<float>("dbscan_gamma", 0.1);
  dbscan_theta = lnh.param<float>("dbscan_theta", 0.1);
  use_dbscan_lines = lnh.param<bool>("use_dbscan_lines", true);

  ROS_INFO("Catenary checker node. Global frame: %s. Base frame: %s",
	   base_frame.c_str(), global_frame.c_str());
  ROS_INFO("Plane dist: %f. Publish pc, marker: %d, %d ", plane_dist, publish_pc, publish_marker);
  ROS_INFO("DBscan epsilon: %f. DBScan min points: %d ", dbscan_epsilon, dbscan_min_points);
  if (use_dbscan_lines) {
    ROS_INFO("Using DBScan Lines. Gamma: %f. Theta: %f", dbscan_gamma, dbscan_theta);
  }

  if (publish_pc) {
    pc_publisher = nh.advertise<sensor_msgs::PointCloud2>("plane_pc", 2, true);
  }

  if (publish_marker) {
    marker_publisher = nh.advertise<visualization_msgs::Marker>("segmented_pc", 2, true);
  }

  ros::spin();

  return 0;
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

std_msgs::ColorRGBA getGray(int num) {
  int i = num % 5;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.b = color.r = color.g = i*0.15 + 0.2;

  if (num < 0)
    color.r = color.b = color.g = 1.0;

  return color;
}


visualization_msgs::Marker pointsToMarker(const std::vector<Point> &points,
					  const std::string frame_id, int n_lines){
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


Obstacle2D toObstacle(const std::vector<Point> &obs) {
  Obstacle2D ret;

  Point2D p;
  for (auto &x:obs) {
    p.x = x.x;
    p.y = x.y;
    ret.push_back(p);
  }

  if (ret.size() > 1)
    ret.calculateConvexHull();

  return ret;
}
