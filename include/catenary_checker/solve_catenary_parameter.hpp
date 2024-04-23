#ifndef __CATENARY_PARAMETERS_HPP__
#define __CATENARY_PARAMETERS_HPP__

#include "ceres/ceres.h"
#include "glog/logging.h"

#include "Eigen/Core"

using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class CatenaryParameters
{
 public:
    CatenaryParameters(double xA_, double yA_, double xB_, double yB_, float l_)
    {
      // Hunging point pA(xA,yA) and pB(xB,yB)
      xA = xA_;
      yA = yA_;
      xB = xB_; 
      yB = yB_; 
      L = l_;
    }

    ~CatenaryParameters(void) 
    {
    }

    template <typename T>
    bool operator()(const T* P_, T* R_) const 
    {
      T x0_ = P_[0];
      T y0_ = P_[1];
      T a_ = P_[2];
      T const_ = T{100.0};
      
      // R_[0] = a_*cosh(xA/a_) - yA;
      // R_[1] = a_*cosh(xB/a_) - yB;
      R_[0] = P_[2] * cosh((xA - P_[0])/P_[2]) + (P_[1] - P_[2]) - yA;
      R_[1] = P_[2] * cosh((xB - P_[0])/P_[2]) + (P_[1] - P_[2]) - yB;
      R_[2] = P_[2]*sinh((xB - P_[0])/P_[2]) - P_[2] * sinh((xA - P_[0])/P_[2]) - L;
      // std::cout <<"x0_= "<< P_[0] << " , y0_=["<< P_[1]<<"] , a_=["<< P_[2]<<"]" <<" , xA="<< xA << " , xB="<< xB <<" , yA="<< yA<<" , yB=" << yB << " , L=" <<L << std::endl;
      // std::cout <<"   R[0]= "<< R_[0] << " , R[1]="<< R_[1]<<" , R[2]="<< R_[2]<<"]" <<std::endl;

      return true;
    }

  private:

    // Point to be evaluated
    double xA, yA, xB, yB, L;
};

class CatenaryParametersSolver
{
  public:

    CatenaryParametersSolver(void);
    ~CatenaryParametersSolver(void);

    bool solve(double _xA, double _yA, double _xB, double _yB, float _l);
    void getCatenaryParameters(double &_x0, double &_y0, double &_a);
    void loadInitialSolution(double p1_, double p2_, double p3_ );
  
    int max_num_iterations;
    double x0, y0, a; 
    double parameter1, parameter2, parameter3;

  private:

};

#endif