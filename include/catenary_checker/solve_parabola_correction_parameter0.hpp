#ifndef __CORRECTION_PARABLE_PARAMETERS_HPP__
#define __CORRECTION_PARABLE_PARAMETERS_HPP__

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"
#include "catenary_checker/grid3d.hpp"

#include <geometry_msgs/Vector3.h>

using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::LossFunction;
using ceres::AutoDiffCostFunction;

struct paramBlockPos{
	double parameter[4];
};

struct paramBlockTether{
	double parameter[4];
};

class CorrectionAutodiffParableLengthFunctor {

public:
    CorrectionAutodiffParableLengthFunctor(){}

	struct ParableLengthFunctor 
	{
	ParableLengthFunctor(double weight_factor, double max_L_)
					: wf(weight_factor), max_L(max_L_)
		{}

		template <typename T>
		bool operator()(const T* const pUGV, const T* const pUAV, const T* const param, T* residual) const 
		{
			T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3]}; // Set first parable point on the reel position
			T dist = T{1.001} * sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2)+pow(pUAV[3]-ugv_reel[3],2)); 
			T maxL = T{max_L};
		
			// Compute parable L : log(q + ((q + 2*p*x)^2 + 1)^(1/2) + 2*p*x)/(4*p) + ((q + 2*p*x)*((q + 2*p*x)^2 + 1)^(1/2))/(4*p) , x = xA and xB
			T X = T{0.0}; // X is 0.0 because is considered that the parable beginning in the ugv reel
			T val = T{2.0}*param[1]*X+param[2]; // This is a common term for the L equation
			T La = (log( param[2] + sqrt((val*val) + T{1.0}) + T{2.0}*param[1]*X)/(T{4.0}*param[1]) + ((val)*sqrt((val*val) + T{1.0}))/(T{4.0}*param[1]));
			
			X = {sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2))};
			val = T{2.0}*param[1]*X+param[2];
			T Lb = (log( param[2] + sqrt((val*val) + T{1.0}) + T{2.0}*param[1]*X)/(T{4.0}*param[1]) + ((val)*sqrt((val*val) + T{1.0}))/(T{4.0}*param[1]));
			T L = Lb - La;

			T diff_;
			if (L < dist )
				diff_ = (dist - L);
			else if (L > maxL)
				diff_ = (L - maxL);
			else
				diff_ = T{0.0};

			residual[0] = wf * 100.0 * (exp(diff_)-1.0) ;
					
			return true;
		}
		double wf, max_L;
	};
private:

};


class CorrectionDistanceFunctionObstacles : public ceres::SizedCostFunction<2, 3> 
{
 public:

    CorrectionDistanceFunctionObstacles(Grid3d* grid_)
      : g_3D(grid_)
    {
    }

	CorrectionDistanceFunctionObstacles(Grid3d* grid_ , Grid3d* grid_obst_ , Grid3d* grid_trav_, double safety_distance_)
      : g_3D(grid_), g_3D_obst(grid_obst_), g_3D_trav(grid_trav_), sd(safety_distance_)
    {
    }

    virtual ~CorrectionDistanceFunctionObstacles(void) 
    {
    }
	
    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
    {
        double x = parameters[0][0];
        double y = parameters[0][1];
        double z = parameters[0][2];
		double d1_, d2_, distance_, type_coll_;
		double min_dist_ = 0.01;

		if(g_3D->isIntoMap(x, y, z)){
			TrilinearParams p = g_3D->getPointDistInterpolation(x, y, z);
            distance_ = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;
			TrilinearParams p1 = g_3D_trav->getPointDistInterpolation(x, y, z);
            d1_ = p1.a0 + p1.a1*x + p1.a2*y + p1.a3*z + p1.a4*x*y + p1.a5*x*z + p1.a6*y*z + p1.a7*x*y*z;
			TrilinearParams p2 = g_3D_obst->getPointDistInterpolation(x, y, z);
            d2_ = p2.a0 + p2.a1*x + p2.a2*y + p2.a3*z + p2.a4*x*y + p2.a5*x*z + p2.a6*y*z + p2.a7*x*y*z;

			if(distance_ < sd){
				if (d1_ < d2_)
					type_coll_ = -1.0;
				else
					type_coll_ = 1.0;
			} else
				type_coll_ = 0.0;

			if(distance_ < min_dist_)
				distance_ = min_dist_;
	
			residuals[0] = distance_;
			residuals[1] = type_coll_;

			if(jacobians != NULL && jacobians[0] != NULL){
				jacobians[0][0] = p.a1 + p.a4*y + p.a5*z + p.a7*y*z;
				jacobians[0][1] = p.a2 + p.a4*x + p.a6*z + p.a7*x*z;
				jacobians[0][2] = p.a3 + p.a5*x + p.a6*y + p.a7*x*y;
			}
		}
        else{
			distance_ = -1.0;
			type_coll_ = -1.0;
			residuals[0] = distance_;
			residuals[1] = type_coll_;
			if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = 0.0;
                jacobians[0][1] = 0.0;
                jacobians[0][2] = 0.0;
            }
        }

        return true;
  }
  private:
	Grid3d *g_3D, *g_3D_obst, *g_3D_trav;
	double sd;
};

class CorrectionNumPointParabola : public ceres::SizedCostFunction<1, 1> 
{
 public:

    CorrectionNumPointParabola(void)
    {
    }

    virtual ~CorrectionNumPointParabola(void) 
    {
    }

    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
    {
		int num_point_per_unit_length = 20;
		
		residuals[0] = round( (double)num_point_per_unit_length * parameters[0][0] );
        if (jacobians != NULL && jacobians[0] != NULL) 
        	jacobians[0][0] = num_point_per_unit_length;

        return true;
  }
  private:

};

class CorrectionAutodiffParableFunctor {

	public:
    CorrectionAutodiffParableFunctor(){}

		struct ParableFunctor 
		{
			ParableFunctor(double wf_, Grid3d* g_3D_, Grid3d* g_3D_obst_, Grid3d* g_3D_trav_, double sb_)
			              :wf(wf_), g_3D(g_3D_), g_3D_obst(g_3D_obst_), g_3D_trav(g_3D_trav_), sb(sb_),
			_parableCostFunctor(new CorrectionDistanceFunctionObstacles(g_3D, g_3D_obst, g_3D_trav, sb)), _numPointFunctor(new CorrectionNumPointParabola())
			{
			}

			template <typename T>
			bool operator()(const T* const pUGV, const T* const pUAV, const T* const param, T* residual) const 
			{
				T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3]}; // Set first parable point on the reel position
				
				// Here is compute the Parable Parameters 
				double min_val_ = 0.01;
				T d_[1];
				T np_; 
				T u_x , u_y;
				bool x_const, y_const;
				x_const = y_const = false;

				T delta_x = (pUAV[1] - ugv_reel[1]);
				T delta_y = (pUAV[2] - ugv_reel[2]);
				T delta_z = (pUAV[3] - ugv_reel[3]);
				T dist_ = sqrt(delta_x * delta_x + delta_y * delta_y );
				if (dist_ < min_val_){
					u_x = u_y = T{0.0};
					d_[0] = sqrt(delta_z*delta_z); 
				} else{
					u_x = delta_x /dist_;
					u_y = delta_y /dist_;
					d_[0] = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z*delta_z);
				}
				if (d_[0] < 1.0) // To not make less than num_point_per_unit_length the value of points in parable
					d_[0] = T{1.0};

        		_numPointFunctor(d_, &np_); // To get the values and parameters needed for computing the parable interpolation

				// Here is compute the parable point and it cost 
				T point[3];
				T dist_to_obst_[2], tetha_;	
				T x_  =  T{0.0};
				std::vector<T> v_dist, v_coll; v_dist.clear(); v_coll.clear();
				std::vector<T> v_p1_, v_p2_, v_p3_; v_p1_.clear(), v_p2_.clear(), v_p3_.clear();
				int first_coll_, last_coll_;
				first_coll_ = last_coll_ = -1;
				for(int i = 0; i < np_; i++, x_ += dist_/ np_ ){  
					if (!(dist_ < min_val_)){ // To check difference position between UGV and UAV only in z-axe, so parable it is not computed
						point[0] = ugv_reel[1] + u_x * x_;
						point[1] = ugv_reel[2] + u_y * x_;
						point[2] = param[1] * x_* x_ + param[2] * x_ + param[3];
					}else{ 	// In case that UGV and UAV are not in the same point the plane X-Y
						T _step = d_[0] / np_;
						point[0] = ugv_reel[1];
						point[1] = ugv_reel[2];
						point[2] = ugv_reel[3]+ _step* (double)i;    	
					}
        			_parableCostFunctor(point, dist_to_obst_);

					if(dist_to_obst_[1] == 1.0){
						if (first_coll_ == -1)
							first_coll_ = i;
						last_coll_ = i;
					}

					v_dist.push_back(dist_to_obst_[0]);
					v_coll.push_back(dist_to_obst_[1]);
					v_p1_.push_back(point[0]);
					v_p2_.push_back(point[1]);
					v_p3_.push_back(point[2]);
				}

				T distance_, C, point_cost_, cost_;
				T cost_state_parable = T{0.0};
				T C1 = T{1.0};
				T C2 = T{20.0};
				T C3 = T{40};
				for(int i = 0; i < np_; i++){ 
					if (v_coll[i]== -1.0){
						if(v_dist[i] < T{0.0}){ //If point out of workspace for grid
							C = C1;
							distance_ = v_p3_[i];
							cost_ = T{-100000.0} * distance_;
						}else{
							C = C2;
							distance_ = v_dist[i];
							cost_ = T{1.0}/distance_;
						}
					}else if(i >=  first_coll_ && i <= last_coll_){
						if(v_coll[i] == 0){
							C = C3;
							distance_ = v_dist[i];
							cost_ = T{1.0/min_val_}*distance_;
						}else{
							C = C2;
							distance_ = v_dist[i];
							cost_ = T{1.0}/distance_;
						}
					}else{
						C = T{0.0};	
						distance_ = v_dist[i];
						cost_ = T{1.0}/distance_;
					}
					point_cost_ = cost_*C;
					cost_state_parable = cost_state_parable + point_cost_; // To get point parable cost
				}
				cost_state_parable = cost_state_parable/np_;
				residual[0] = wf * cost_state_parable;
				return true;
			}
			double wf, sb;
			std::string user;
			Grid3d *g_3D, *g_3D_obst, *g_3D_trav;
	    ceres::CostFunctionToFunctor<2, 3> _parableCostFunctor;
	    ceres::CostFunctionToFunctor<1, 1> _numPointFunctor;
		};
	private:
};


class CorrectionAutodiffParableParametersFunctor {

public:
    CorrectionAutodiffParableParametersFunctor(){}

	struct ParableParametersFunctor 
	{
	ParableParametersFunctor(double weight_factor): wf(weight_factor)
		{}

		template <typename T>
		bool operator()(const T* const pUGV, const T* const pUAV, const T* const param, T* residual) const 
		{
			T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3]}; // Set first parable point on the reel position
		
			T Xa = T{0.0};
			T Xb = sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2)); 

			T pow_a = (param[1]*Xa*Xa + param[2]*Xa + param[3] - ugv_reel[3]);// residual = p*xa² + q*xa + r - ya
			T pow_b = (param[1]*Xb*Xb + param[2]*Xb + param[3] - pUAV[3]    );// residual = p*xb² + q*xb + r - yb

			residual[0] = (wf * 100.0 )* ((pow_a));
			residual[1] = (wf * 100.0 )* ((pow_b));

			return true;
		}
		
		double wf;
	};

private:

};


class CorrectionParabolaParametersSolver
{
  public:

    CorrectionParabolaParametersSolver(Grid3d* g_3D_, Grid3d* g_3D_obst_, Grid3d* g_3D_trav_, double d_tether_obs_, double l_tether_max_, int i, geometry_msgs::Vector3 plr_);
    ~CorrectionParabolaParametersSolver(void);

    bool solve(double w_1_, double w_2_, double w_3_);
    void loadInitialStatus(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, double pa_, double pb_, double pc_);
    void getParabolaParameters(double &_p, double &_q, double &_r);
  
    int max_num_iterations, status;
    double p, q, r; 
    double distance_tether_obstacle, length_tether_max;
    geometry_msgs::Vector3 p_ugv, p_uav, p_local_reel;
    Grid3d *grid_3D, *grid_3D_obst, *grid_3D_trav;

    paramBlockPos statesPosUGV, statesPosUAV;
	paramBlockTether statesTetherParams;

  private:

};

#endif