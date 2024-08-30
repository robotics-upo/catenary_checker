#include "catenary_checker/solve_catenary_correction_parameter.hpp"


CorrectionCatenaryParametersSolver::CorrectionCatenaryParametersSolver(Grid3d* g_3D_, double d_tether_obs_, double l_tether_max_, int i_, geometry_msgs::Vector3 plr_) 
{
  // google::InitGoogleLogging("CatenaryCeresSolver");
  max_num_iterations = 500;
  grid_3D = g_3D_;
  distance_tether_obstacle = d_tether_obs_;
  length_tether_max = l_tether_max_;
  status = i_;
  p_local_reel = plr_;
} 

CorrectionCatenaryParametersSolver::~CorrectionCatenaryParametersSolver(void)
{
} 

void CorrectionCatenaryParametersSolver::loadInitialStatus(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_, double pa_, double pb_, double pc_)
{
  statesTetherParams.parameter[0] = 0;  
  statesTetherParams.parameter[1] = pa_;  
  statesTetherParams.parameter[2] = pb_;  
  statesTetherParams.parameter[3] = pc_;
  std::cout << "  Loading initial parameter por optimation:["<< pa_ << "," << pb_ << "," << pc_ <<"]"<<std::endl;
  statesPosUGV.parameter[0] = 0;  
  statesPosUGV.parameter[1] = p1_.x;  
  statesPosUGV.parameter[2] = p1_.y;  
  statesPosUGV.parameter[3] = p1_.z;
  statesPosUAV.parameter[0] = 0;  
  statesPosUAV.parameter[1] = p2_.x;  
  statesPosUAV.parameter[2] = p2_.y;  
  statesPosUAV.parameter[3] = p2_.z;
} 


bool CorrectionCatenaryParametersSolver::solve(double w_1_, double w_2_, double w_3_)
{
  // Build the problem.
  Problem problem;
	LossFunction* loss_function = NULL;

  // Set up a cost funtion per point into the cloud

	// CostFunction* cf1  = new AutoDiffCostFunction<CorrectionAutodiffCatenaryFunctor::CatenaryFunctor, 1, 4, 4, 4> //Residual, ugvPos, uavPos, catenaryParams
	// (new CorrectionAutodiffCatenaryFunctor::CatenaryFunctor(w_1_, grid_3D, grid_3D_obst, grid_3D_trav, distance_tether_obstacle)); 
  CostFunction* cf1  = new AutoDiffCostFunction<CorrectionAutodiffCatenaryFunctor::CatenaryFunctor, 1, 4, 4, 4> //Residual, ugvPos, uavPos, catenaryParams
	(new CorrectionAutodiffCatenaryFunctor::CatenaryFunctor(w_1_, grid_3D, distance_tether_obstacle)); 
	  problem.AddResidualBlock(cf1, loss_function, statesPosUGV.parameter, statesPosUAV.parameter, statesTetherParams.parameter);
	  problem.SetParameterLowerBound(statesTetherParams.parameter, 1, 0.0);
	  problem.SetParameterLowerBound(statesTetherParams.parameter, 3, p_local_reel.z);
	  problem.SetParameterBlockConstant(statesPosUGV.parameter);
	  problem.SetParameterBlockConstant(statesPosUAV.parameter);
	
  CostFunction* cf2  = new AutoDiffCostFunction<CorrectionAutodiffCatenaryLengthFunctor::CatenaryLengthFunctor, 1, 4, 4, 4>
	(new CorrectionAutodiffCatenaryLengthFunctor::CatenaryLengthFunctor(w_2_, length_tether_max)); 
	  problem.AddResidualBlock(cf2, loss_function, statesPosUGV.parameter, statesPosUAV.parameter, statesTetherParams.parameter);
	  problem.SetParameterLowerBound(statesTetherParams.parameter, 1, 0.0);
	  problem.SetParameterLowerBound(statesTetherParams.parameter, 3, p_local_reel.z);
	  problem.SetParameterBlockConstant(statesPosUGV.parameter);
	  problem.SetParameterBlockConstant(statesPosUAV.parameter);
    
	CostFunction* cf3  = new AutoDiffCostFunction<CorrectionAutodiffCatenaryParametersFunctor::CatenaryParametersFunctor, 2, 4, 4, 4>
	(new CorrectionAutodiffCatenaryParametersFunctor::CatenaryParametersFunctor(w_3_)); 
	  problem.AddResidualBlock(cf3, loss_function, statesPosUGV.parameter, statesPosUAV.parameter, statesTetherParams.parameter);
	  problem.SetParameterLowerBound(statesTetherParams.parameter, 1, 0.0);
	  problem.SetParameterLowerBound(statesTetherParams.parameter, 3, p_local_reel.z);
	  problem.SetParameterBlockConstant(statesPosUGV.parameter);
	  problem.SetParameterBlockConstant(statesPosUAV.parameter);

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = max_num_iterations;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  if(summary.message == "Initial residual and Jacobian evaluation failed.")
    printf("\t\t <<<< Failed in status number: [%i] >>>>\n", status);

  // Some debug information
  // std::cout << summary.BriefReport() << "\n";

  // Get the solution
  p = statesTetherParams.parameter[1]; q = statesTetherParams.parameter[2]; r = statesTetherParams.parameter[3]; 

  // std::cout <<"  p: " << p << " , q: " << q << " , r: "<< r << std::endl;
      
  return true; 
}

void CorrectionCatenaryParametersSolver::getParabolaParameters(double &_p, double &_q, double &_r)
{
  _p = p;
  _q = q;
  _r = r;
}



