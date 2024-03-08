#pragma once

#include <catenary_checker/point_2d.hpp>
#include <catenary_checker/obstacle_2d.hpp>
#include <catenary_checker/catenary_checker.hpp>
#include <ros/ros.h>

class PreprocessedScenario {
public:
  PreprocessedScenario(Point2D min, Point2D max, int n_theta, float plane_dist = 0.2,
                       int dbscan_min_points = 20, float dbscan_epsilon = 0.05);

  // ! @brief Constructor that gets the parameters from ROS
  PreprocessedScenario(ros::NodeHandle &pnh);

  //! @brief Translates a Scenario given by a PC 3D to a matrix of 2D scenarios
  //! @brief by slicing them in different directions
  void precompute(const pcl::PointCloud<pcl::PointXYZ> &pc);

  std::vector<TwoPoints> getProblemsTheta(double theta) const;

  bool exportScenarios(const std::string &name) const;

  //! @brief Checks whether there exists a catenary between A and B with
  //! @brief a maximum length.
  //! @param A Starting point (3D)
  //! @param B Final point (3D)
  //! @param max_length Maximum tether length
  //! @retval -1.0 No valid catenary
  //! @return The length of the collision-free catenary
  float checkCatenary(const pcl::PointXYZ &A,
                      const pcl::PointXYZ &B,
                      double max_length) const;

  std::vector<std::vector<Scenario> > _scenarios;
  std::vector<std::vector<TwoPoints> > _problems;

  Point2D _min, _max;
  int _n_theta;
  float _plane_dist;
  float _max_z = 10.0f;

  // DBScan stuff
  int _db_min_points;
  float _db_epsilon;
};
