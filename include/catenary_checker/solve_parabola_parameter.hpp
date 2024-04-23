#ifndef __PARABLE_PARAMETERS_HPP__
#define __PARABLE_PARAMETERS_HPP__

#include "ceres/ceres.h"
#include "glog/logging.h"

#include "Eigen/Core"

using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class ParabolaParameters
{
 public:
    ParabolaParameters(double xA_, double yA_, double xB_, double yB_, double a_)
    {
      xA = xA_;
      yA = yA_;
      xB = xB_; 
      yB = yB_; 
      A = a_;
    }

    ~ParabolaParameters(void) 
    {
    }

    template <typename T>
    bool operator()(const T* P_, T* R_) const 
    {
      T p = P_[0];
      T q = P_[1];
      T r = P_[2];
      
      R_[0] = p * pow(xA,2) + q * xA + r - yA;
      R_[1] = p * pow(xB,2) + q * xB + r - yB;
      R_[2] = p * pow(xB,3)/T{3.0} + q * pow(xB,2)/T{2.0} + r * xB - (p * pow(xA,3)/T{3.0} + q * pow(xA,2)/T{2.0} + r * xA ) - A;

      return true;
    }

  private:

    // Point to be evaluated
    double xA, yA, xB, yB;
    // Parabola area
    double A;
};

class ParabolaParametersSolver
{
  public:

    ParabolaParametersSolver(void);
    ~ParabolaParametersSolver(void);

    bool solve(double _xA, double _yA, double _xB, double _yB, double _A, int _i);
    void getParabolaParameters(double &_p, double &_q, double &_r);
    void loadInitialSolution(double p1_, double p2_, double p3_ );
  
    int max_num_iterations;
    double p, q, r; 
    double parameter1, parameter2, parameter3;

  private:

};

#endif