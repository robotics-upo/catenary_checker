#include "catenary_checker/solve_parable_parameter.hpp"


ParableParametersSolver::ParableParametersSolver(void) 
{
  // google::InitGoogleLogging("CatenaryCeresSolver");
  max_num_iterations = 500;
  parameter1 = 1.0;
  parameter2 = 0.5;
  parameter3 = 0.5;
}

ParableParametersSolver::~ParableParametersSolver(void)
{
} 

void ParableParametersSolver::loadInitialSolution(double p1_, double p2_, double p3_ )
{
  parameter1 = p1_;
  parameter2 = p2_;
  parameter3 = p3_;
} 


bool ParableParametersSolver::solve(double _xA, double _yA, double _xB, double _yB, double _A)
{
  // Initial solution
  double x[3];
  x[0] = parameter1;
  x[1] = parameter2;
  x[2] = parameter3;
          
   // Build the problem.
   Problem problem;

  // Set up a cost funtion per point into the cloud
  CostFunction* cf1 = new ceres::AutoDiffCostFunction<ParableParameters, 3, 3>( new ParableParameters(_xA, _yA, _xB, _yB, _A) );
  problem.AddResidualBlock(cf1, NULL, x);

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = max_num_iterations;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  // Some debug information
  // std::cout << summary.BriefReport() << "\n";

  // Get the solution
  p = x[0]; q = x[1]; r = x[2]; 

  // std::cout <<"  p: " << p << " , q: " << q << " , r: "<< r << std::endl;
      
  return true; 
}

void ParableParametersSolver::getParableParameters(double &_p, double &_q, double &_r)
{
  _p = p;
  _q = q;
  _r = r;
}



