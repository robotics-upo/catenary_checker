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
    CatenaryParameters(double xA_, double yA_, double xB_, double yB_, float l_, float dist_, float maxL_)
    {
      // Hunging point pA(xA,yA) and pB(xB,yB)
      xA = xA_;
      yA = yA_;
      xB = xB_; 
      yB = yB_; 
      L = l_;
      D = dist_;
      maxL = maxL_;
    }

    ~CatenaryParameters(void) 
    {
    }

    template <typename T>
    bool operator()(const T* P_, T* R_) const 
    {
      T x0_ = P_[1];
      T y0_ = P_[2];
      T a_ = P_[3];
      float wf_ = 100.0;
      
      // R_[0] = a_*cosh(xA/a_) - yA;
      // R_[1] = a_*cosh(xB/a_) - yB;
      R_[0] = P_[3] * cosh((xA - P_[1])/P_[3]) + (P_[2] - P_[3]) - yA;
      R_[1] = P_[3] * cosh((xB - P_[1])/P_[3]) + (P_[2] - P_[3]) - yB;
      T length = P_[3] * sinh((xB - P_[1])/P_[3]) - P_[3] * sinh((xA - P_[1])/P_[3]);
      R_[2] = T{wf_}*length - L;

			T diff_;
			if (length < D)
				diff_ = T{D - maxL};
			else if (length > maxL)
				diff_ = T{length - maxL};
			else
				diff_ = T{0.0};

			R_[3] = T{wf_}*(exp(diff_)-1.0);

      // std::cout <<"       ["<< P_[0]<<"]  x0_= "<< P_[1] << " , y0_=["<< P_[2]<<"] , a_=["<< P_[3]<<"]" <<" , xA="<< xA << " , xB="<< xB <<" , yA="<< yA<<" , yB=" << yB << " , L=" <<L << std::endl;
      // std::cout <<"       R[0]= "<< R_[0] << " , R[1]="<< R_[1]<<" , R[2]="<< R_[2]<<"]" <<std::endl;

      return true;
    }

  private:

    // Point to be evaluated
    double xA, yA, xB, yB, L, D, maxL;
};

class CatenaryParametersSolver
{
  public:

    CatenaryParametersSolver(void);
    ~CatenaryParametersSolver(void);

    bool solve(double _xA, double _yA, double _xB, double _yB, float _l, float _dist, float _maxL, float _upper_bound);
    void getCatenaryParameters(double &_x0, double &_y0, double &_a);
    void loadInitialSolution(int i_, double p1_, double p2_, double p3_ );
  
    int max_num_iterations;
    double x0, y0, a; 
    double parameter0, parameter1, parameter2, parameter3;

  private:

};

#endif