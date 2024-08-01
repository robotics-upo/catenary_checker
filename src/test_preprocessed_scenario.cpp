#include "catenary_checker/preprocessed_scenario.hpp"
#include <string>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

PreprocessedScenario *ps = NULL;
ros::Publisher pub, pub_marker;

using namespace std;
using namespace std::chrono;

int main(int argc, char **argv) {
  // Init ROS
  ros::init(argc,argv, "test_preprocessed_scenario");


  // Test the scenario preprocessing
  std::string file = "scenarios.tar.gz";
  if (argc > 1) {
    file = argv[1];
  }
  auto st = chrono::system_clock::now();
  ps = new PreprocessedScenario(file);
  auto end = chrono::system_clock::now();
  duration<float, std::milli> duration = end - st;
  ROS_INFO("Got scenarios. Expended time: %f s", duration.count() * milliseconds::period::num / milliseconds::period::den);
  ROS_INFO("Scenarios statistics: %s", ps->getStats().c_str() );

  // Feature: show different scenarios present in the environment

  ros::Rate loop_rate(0.2);
  int cont = 0;
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    if (cont >= ps->size()) {
      cont = 0;
    }
    ps->publishScenarios(cont++);
  }

  delete ps;
  ps = NULL;
}

