#include "catenary_checker/solve_parable_parameter.hpp"


ParableParametersSolver::ParableParametersSolver(void) 
{
  // google::InitGoogleLogging("CatenaryCeresSolver");
  max_num_iterations = 400;
}

ParableParametersSolver::~ParableParametersSolver(void)
{
} 


bool ParableParametersSolver::solve(double _xA, double _yA, double _xB, double _yB, double _A)
{
  // Initial solution
  double x[3];
  x[0] = 1.0;
  x[1] = 0.5;
  x[2] = 0.5;
          
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
  std::cout << summary.BriefReport() << "\n";

  // Get the solution
  p = x[0]; q = x[1]; r = x[2]; 

  std::cout <<"  p: " << p << " , q: " << q << " , r: "<< r << std::endl;
      
  return true; 
}

void ParableParametersSolver::getParableParameters(double &_p, double &_q, double &_r)
{
  _p = p;
  _q = q;
  _r = r;
}



