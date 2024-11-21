#include <string>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <catenary_checker/catenary_checker_node.h>
#include "catenary_checker/scenario.hpp"


using namespace std;

PreprocessedScenario *ps = NULL;
ros::Publisher pub, pub_marker;

using namespace std;
using namespace std::chrono;

int main(int argc, char **argv)
{
    std::string node_name = "test_scenario";

	ros::init(argc, argv, node_name);

    Grid3d grid_3D(node_name);

    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    // Prepare the checker
    catenaryChecker cc(nh); 
    Point3D A(0.0f, 1.0f, 0.0f);
    Point3D B(-5.0f, 1.0f, 4.0f);

    if (argc > 3) {
        A.x = atof(argv[1]);
        A.y = atof(argv[2]);
        A.z = atof(argv[3]);
    }
    if (argc > 6) {
        B.x = atof(argv[4]);
        B.y = atof(argv[5]);
        B.z = atof(argv[6]);        
    }

    double min_dist = 0.3;
    if (argc > 7) {
        min_dist = atof(argv[7]);
    }

    Scenario s(grid_3D, A, B, min_dist, 0.05);

    ROS_INFO("Testing from A = %s to B = %s \t Scenario generated. Size: %lu", A.toString().c_str(), B.toString().c_str(), s.size());

    ros::Publisher marker_publisher = nh->advertise<visualization_msgs::MarkerArray>("scenario", 2, true);

    marker_publisher.publish(s.toMarkerArray("map", 0, true));

    ros::spin();

    return 0;
}
