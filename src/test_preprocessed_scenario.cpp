#include <string>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <catenary_checker/catenary_checker_node.h>
#include "catenary_checker/preprocessed_scenario.hpp"


using namespace std;

PreprocessedScenario *ps = NULL;
ros::Publisher pub, pub_marker;

using namespace std;
using namespace std::chrono;

int main(int argc, char **argv) {
  // Init ROS
  ros::init(argc,argv, "test_preprocessed_scenario");

  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  catenaryChecker cc(nh);

  
  // Test the scenario preprocessing
  string file = "scenarios.tar.gz";
  if (argc > 1) {
    file = argv[1];
  
    auto st = chrono::system_clock::now();
    ps = new PreprocessedScenario(file);
    auto end = chrono::system_clock::now();
    duration<float, std::milli> duration = end - st;
    ROS_INFO("Got scenarios. Expended time: %f s", duration.count() * milliseconds::period::num / milliseconds::period::den);
    ROS_INFO("Scenarios statistics: %s", ps->getStats().c_str() );

    // Feature: show different scenarios present in the environment
    ros::Rate loop_rate(0.25);
    int cont = 0;
    pcl::PointXYZ A, B;

    if (argc < 3) {
      while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        ps->publishScenarios(cont++);
      
      }
    } else {
      int cont = 0;
      // 
      for (;cont < 5 && ros::ok(); cont ++) {
        ros::spinOnce();
        loop_rate.sleep();
      }

      cc.getPointCloud(ps->_pc);
 
      while(ros::ok()) {
        cout << "Please enter A: ";
        cin >> A.x >> A.y >> A.z;
        cout << "Please enter B: ";
        cin >> B.x >> B.y >> B.z;
        std::vector<geometry_msgs::Point> vp, vp2;
        geometry_msgs::Point p1, p2;
        double length_cat = ps->checkCatenary(A, B, vp, true);
        if ( length_cat > -0.5f) {
          cout << "Preprocessed: There exists a catenary. Length = " << length_cat << endl;
        } else {
          cout << "Preprocessed: Could not find catenary" << endl;
        }

        p1.x = A.x; p1.y = A.y; p1.z = A.z;
        p2.x = B.x; p2.y = B.y; p2.z = B.z;

        if (cc.analyticalCheckCatenary(p1, p2, vp2)) {
          cout << "Live: There exists a catenary" << endl;
        } else {
          cout << "Live: There not exists a catenary" << endl;
        
        }
        sleep(1);
        ros::spinOnce();
      }
    }
  } else {
    ps = new PreprocessedScenario();
    ros::Rate loop_rate(0.5);
    int cont = 0;
    
    while(ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
      ps->publishProblems(cont++);
    }

    delete ps;
    ps = NULL;
  }
}

