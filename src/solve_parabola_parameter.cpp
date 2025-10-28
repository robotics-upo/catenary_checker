#include "catenary_checker/solve_parabola_parameter.hpp"


ParabolaParametersSolver::ParabolaParametersSolver(void) 
{
  // google::InitGoogleLogging("CatenaryCeresSolver");
  max_num_iterations = 500;
  parameter1 = 1.0;
  parameter2 = 0.5;
  parameter3 = 0.5;
}

ParabolaParametersSolver::~ParabolaParametersSolver(void)
{
} 

void ParabolaParametersSolver::loadInitialSolution(double p1_, double p2_, double p3_ )
{
  parameter1 = p1_;
  parameter2 = p2_;
  parameter3 = p3_;
} 


bool ParabolaParametersSolver::solve(double _xA, double _yA, double _xB, double _yB, double _A, int _i)
{
  // Initial solution
  double x[3];
  x[0] = parameter1;
  x[1] = parameter2;
  x[2] = parameter3;
          
   // Build the problem.
   Problem problem;

  // Set up a cost funtion per point into the cloud
  CostFunction* cf1 = new ceres::AutoDiffCostFunction<ParabolaParameters, 3, 3>( new ParabolaParameters(_xA, _yA, _xB, _yB, _A) );
  problem.AddResidualBlock(cf1, NULL, x);
  problem.SetParameterLowerBound(x, 0, 0.0);
  problem.SetParameterUpperBound(x, 0, 50.0);
  problem.SetParameterLowerBound(x, 2, 0.38);

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = max_num_iterations;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  if(summary.message == "Initial residual and Jacobian evaluation failed.")
    printf("\t\t <<<< Failed in status number: [%i] >>>>\n",_i);

  // Some debug information
  // std::cout << summary.BriefReport() << "\n";

  // Get the solution
  p = x[0]; q = x[1]; r = x[2]; 

  // std::cout <<"  p: " << p << " , q: " << q << " , r: "<< r << std::endl;
      
  return true; 
}

void ParabolaParametersSolver::getParabolaParameters(double &_p, double &_q, double &_r)
{
  _p = p;
  _q = q;
  _r = r;
}



