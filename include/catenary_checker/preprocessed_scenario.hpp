#pragma once

#include <catenary_checker/point_2d.hpp>
#include <catenary_checker/obstacle_2d.hpp>
#include <catenary_checker/catenary_checker.hpp>
#include <ros/ros.h>
#include <iostream>

class PreprocessedScenario {
public:
  //! @brief Constructor from file
  PreprocessedScenario(const std::string &file);

  //! @brief Checks whether there exists a catenary between A and B with
  //! @brief a maximum length.
  //! @param A Starting point (3D)
  //! @param B Final point (3D)
  //! @retval -1.0 No valid catenary
  //! @return The length of the collision-free catenary. T
  //! @note The x coordinates of the 2D points and the last plane are stored in the class
  float checkCatenary(const pcl::PointXYZ &A, const pcl::PointXYZ &B);

  inline std::string getStats() const {
    std::ostringstream oss;

    oss << "Number of scenarios: " << _scenarios.size() << std::endl;
    if (_scenarios.size() > 0) {
      oss << "Number of obstacles of scenario " << _scenarios.size()/2 << std::endl;
      oss << " is " << _scenarios[_scenarios.size() / 2].size() << std::endl;
    }

    return oss.str();
  }

  void publishScenarios(unsigned int scen);

  size_t size(){return _scenarios.size();}

  //! @brief Stores the last data of parable
  Parable _parable;  
  Point2D _pa, _pb; // X coordinates of each extreme point of the last parable
  PlaneParams _last_plane;

protected:
  //! @brief Translates a Scenario given by a PC 3D to a matrix of 2D scenarios
  //! @brief by slicing them in different directions
  void precompute(const pcl::PointCloud<pcl::PointXYZ> &pc);

  std::vector<TwoPoints> getProblemsTheta(double theta) const;

  bool exportScenario() const;

  //! @brief Loads the precomputed scenarios from a tar gz file
  //! @param file The global path to file
  //! @retval true If success
  bool loadScenario(const std::string &file);

  //! @brief Gets the metadata from a YAML file
  bool getMetadata(const std::string &f);

  std::vector<std::vector<Scenario> > _scenarios;
  std::vector<std::vector<TwoPoints> > _problems;
  std::string _filename;

public:
  Point2D _min, _max;
  int _n_theta;
  float _plane_dist;
  float _max_z = 10.0f;

protected:

  // DBScan stuff
  int _db_min_points;
  float _db_epsilon;

  std::string getFilename(const std::string &f) const;


  // ROS Stuff
  ros::Subscriber _sub;
  ros::Publisher _pub, _pub_marker;

public:
  void PC_Callback(const sensor_msgs::PointCloud2::ConstPtr &pc);

};
