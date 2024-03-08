#include "catenary_checker/preprocessed_scenario.hpp"
#include <string>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

PreprocessedScenario *ps = NULL;
ros::Publisher pub, pub_marker;

void pc_callback(const sensor_msgs::PointCloud2::ConstPtr &pc);

int n = 5;

int main(int argc, char **argv) {
  // Init ROS
  ros::init(argc,argv, "test_preprocessed_scenario");
  ros::NodeHandle nh, pnh("~");
  ros::Subscriber sub = nh.subscribe("point_cloud", 1, pc_callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("slices", 199, true);
  pub_marker = nh.advertise<visualization_msgs::MarkerArray>("scenarios", 2, true);

  // Test the scenario preprocessing
  ps = new PreprocessedScenario(nh);

  if (argc > 1) {
    n = atoi(argv[1]);
  }

  ros::Rate loop_rate(1);
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete ps;
  ps = NULL;
}


using namespace std;
using namespace std::chrono;
void pc_callback(const sensor_msgs::PointCloud2::ConstPtr &pc) {
  pcl::PointCloud<pcl::PointXYZ> pcl_pc;
  if (ps != NULL) {
    pcl::fromROSMsg(*pc, pcl_pc);

    ROS_INFO("Received PointCloud. Starting preprocessing. Height, Width: %d, %d", pc->height, pc->width);

    auto st = chrono::system_clock::now();
    ps->precompute(pcl_pc);
    auto end = chrono::system_clock::now();
    duration<float, std::milli> duration = end - st;
    ROS_INFO("Expended time: %f s", duration.count() * milliseconds::period::num / milliseconds::period::den);

    ROS_INFO("Exporting scenario.");
    ps->exportScenarios("scenarios");

    if (ps->_scenarios.size() > 0) {
      ROS_INFO("Publishing first scenarios.");

      int i = 0;
      int total = ps->_scenarios[n].size();
      for (auto &x:ps->_scenarios[n]) {
        if (x.getTotalPoints() > 200) {
          float intensity = 0.5f + 0.5f/(static_cast<float>(i) / static_cast<float>(total));
          pub.publish(x.toPC("map", i, intensity));
          pub_marker.publish(x.toMarkerArray("map", i++));
        }
      }
    }
  }
}
