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

class CorrectionDistanceFunctionObstacles : public ceres::SizedCostFunction<1, 3> 
{
 public:

	CorrectionDistanceFunctionObstacles(Grid3d* grid_): g_3D(grid_)
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
		double d_;
		double min_dist_ = 0.005;

		if(g_3D->isIntoMap(x, y, z))
        {
			TrilinearParams p = g_3D->getPointDistInterpolation(x, y, z);
            d_ = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;

			if(d_ < min_dist_)
				d_ = min_dist_;

			residuals[0] = d_;

			if(jacobians != NULL && jacobians[0] != NULL){
				jacobians[0][0] = p.a1 + p.a4*y + p.a5*z + p.a7*y*z;
				jacobians[0][1] = p.a2 + p.a4*x + p.a6*z + p.a7*x*z;
				jacobians[0][2] = p.a3 + p.a5*x + p.a6*y + p.a7*x*y;
			}

		}
        else
        {
			d_ = -1.0;
			residuals[0] = d_;
			if (jacobians != NULL && jacobians[0] != NULL && jacobians[1] != NULL) 
            {
                jacobians[0][0] = 0.0;
                jacobians[0][1] = 0.0;
                jacobians[0][2] = 0.0;
            }
        }

        return true;
  }
  private:
	Grid3d *g_3D;
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
			ParableFunctor(double weight_factor_, Grid3d* grid_3D_, double sb_)
			: wf(weight_factor_), g_3D(grid_3D_), sb(sb_),_parableCostFunctor(new CorrectionDistanceFunctionObstacles(g_3D)), _numPointFunctor(new CorrectionNumPointParabola())
			{
			}

			template <typename T>
			bool operator()(const T* const pUGV, const T* const pUAV, const T* const param, T* residual) const 
			{
				T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3]}; // Set first parable point on the reel position
				
				// Here is compute the Parable Parameters 
				double min_val_ = 0.01;
				double min_dist_ = 0.005;
				T d_[1];
				T np_, u_x , u_y;
				bool x_const, y_const;
				x_const = y_const = false;
				std::vector<int> v_coll;

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
				T dist_to_obst_;	
				T x_  =  T{0.0};
				std::vector<T> v_dist ; v_dist.clear();
				std::vector<T> v_x_, v_y_, v_z_; v_x_.clear(); v_y_.clear(); v_z_.clear();
				std::vector<T> vx_coll_, vy_coll_, vz_coll_; vx_coll_.clear(); vy_coll_.clear(); vz_coll_.clear();

				std::vector<int> v_transition_coll_; v_transition_coll_.clear();
				int first_coll_, last_coll_, prev_coll_;
				first_coll_ = last_coll_ = prev_coll_ = -2;
				for(int i = 0; i < np_; i++, x_ += dist_/ np_ ){  
					if (!(dist_ < min_val_)){ // To check difference position between UGV and UAV only in z-axe, so parable it is not computed
						point[0] = ugv_reel[1] + u_x * x_;
						point[1] = ugv_reel[2] + u_y * x_;
						point[2] = param[1] * x_* x_ + param[2] * x_ + param[3];
					}
					else{ 	// In case that UGV and UAV are not in the same point the plane X-Y
						T _step = d_[0] / np_;
						point[0] = ugv_reel[1];
						point[1] = ugv_reel[2];
						point[2] = ugv_reel[3]+ _step* (double)i;    	
					}
        			_parableCostFunctor(point, &dist_to_obst_);
					// Data save for aware of the tether status
					v_dist.push_back(dist_to_obst_);
					v_x_.push_back(point[0]);
					v_y_.push_back(point[1]);
					v_z_.push_back(point[2]);
					// Clasification of collision with floor or other obstacles
					if(dist_to_obst_ < T(0.0) || pUGV[3] >= point[2])
						v_coll.push_back(-1);
					else if (dist_to_obst_ < T(sb))
						v_coll.push_back(1);
					else
						v_coll.push_back(0);

					// Algorithm to check intervals points in collision
					if(v_coll[i] == 1 && prev_coll_ == 0){
						first_coll_ = i;
						prev_coll_ = 1;
					}else if( v_coll[i] == 0 && prev_coll_ == 1 && first_coll_!=2){
						last_coll_ = i-1;
						prev_coll_ = 0;
					}else if (v_coll[i] == 1){
						prev_coll_ = 1;
					}else if (v_coll[i] == 0){
						prev_coll_ = 0;
					}else if (v_coll[i] == -1){
						prev_coll_ = -1;
						first_coll_ = last_coll_ = -2;
					}
					if (first_coll_ != -2 && last_coll_ != -2){
						v_transition_coll_.push_back(first_coll_);
						v_transition_coll_.push_back(last_coll_);
						first_coll_ = last_coll_ = -2;
						vx_coll_.push_back(v_x_[first_coll_]);
						vy_coll_.push_back(v_y_[first_coll_]);
						vz_coll_.push_back(v_z_[first_coll_]);
						vx_coll_.push_back(v_x_[last_coll_]);
						vy_coll_.push_back(v_y_[last_coll_]);
						vz_coll_.push_back(v_z_[last_coll_]);
					}
				}
				int group_coll_, first_coll , last_coll , middle_coll;
				T m1_, m2_, Cmax;
				group_coll_ = first_coll = last_coll = middle_coll = -1;
				if(v_transition_coll_.size() > 0){ // this vector only is fill when the are tether points far from collision (v_coll = 0), even though tether is in collision
					group_coll_ = (v_transition_coll_.size()/2);
					first_coll = v_transition_coll_[0];
					last_coll = v_transition_coll_[v_transition_coll_.size()-1];
					middle_coll = ceil((last_coll+first_coll)/2);
					double count_;
					for(int i = 0; i < np_; i++){ 
						if (i >= first_coll && i <= last_coll && v_coll[i]== 0 )
							count_++;
					}
					double max_cost_ = 200.0 + 20.0 * count_;
					Cmax = T{max_cost_};
					T min_cost1_ = T{1.0}/ v_dist[first_coll];
					T min_cost2_ = T{1.0}/ v_dist[last_coll];
					double diff1_ = double(middle_coll- first_coll);
					double diff2_ = double(last_coll- middle_coll);
						if (diff1_ ==0)
							diff1_ = 1;
						if (diff2_ ==0)
							diff2_ = 1;
					m1_ = ( (Cmax - min_cost1_)/ T{diff1_} );
					m2_ = ( (min_cost2_ - Cmax)/ T{diff2_} );
				}

				T distance_, C, point_cost_, cost_;
				T cost_state_parable = T{0.0};
				for(int i = 0; i < np_; i++){ 
					if (v_coll[i]== -1){
						if (v_dist[i] > 0){
							C = T{10.0};
							distance_ = v_dist[i];
							cost_ = T{1.0}/distance_;
						}
						else{
							C = T{1.0};
							distance_ = T{-3.0};
							cost_ = T{10000.0};	
						}
					}else if (v_coll[i]== 1){
						if (i >= first_coll && i <= middle_coll && v_transition_coll_.size() > 0){
							C = T{20.0};
							double diff_x_ = double(i - first_coll);
							distance_ = T{diff_x_};
							cost_ =  m1_ * T{diff_x_} + T{1.0}/ v_dist[first_coll];
						}else if (i > middle_coll && i <= last_coll && v_transition_coll_.size() > 0){
							C = T{20.0};
							double diff_x_ = double(i - middle_coll);
							distance_ = T{diff_x_};
							cost_ =  m2_ * T{diff_x_} + Cmax;
						}
						else{
							C = T{10.0};
							distance_ = v_dist[i];
							cost_ = T{1.0}/distance_;
						}
					}
					else{
						C = T{1.0};
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
			Grid3d *g_3D;
	    	ceres::CostFunctionToFunctor<1, 3> _parableCostFunctor;
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

    CorrectionParabolaParametersSolver(Grid3d* g_3D_, double d_tether_obs_, double l_tether_max_, int i, geometry_msgs::Point plr_);
    ~CorrectionParabolaParametersSolver(void);

    bool solve(double w_1_, double w_2_, double w_3_);
    void loadInitialStatus(geometry_msgs::Point p1_, geometry_msgs::Point p2_, double pa_, double pb_, double pc_);
    void getParabolaParameters(double &_p, double &_q, double &_r);
  
    int max_num_iterations, status;
    double p, q, r; 
    double distance_tether_obstacle, length_tether_max;
    geometry_msgs::Point p_ugv, p_uav, p_local_reel;
    Grid3d *grid_3D, *grid_3D_obst, *grid_3D_trav;

    paramBlockPos statesPosUGV, statesPosUAV;
	paramBlockTether statesTetherParams;

  private:

};

#endif