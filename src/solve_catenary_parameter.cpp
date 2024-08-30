#include "catenary_checker/solve_catenary_parameter.hpp"


CatenaryParametersSolver::CatenaryParametersSolver(void) 
{
  // google::InitGoogleLogging("CatenaryCeresSolver");
  max_num_iterations = 1000;
  parameter1 = 0.1; // x0
  parameter2 = 0.1; // y0
  parameter3 = 0.1; // a
}

CatenaryParametersSolver::~CatenaryParametersSolver(void)
{
} 

void CatenaryParametersSolver::loadInitialSolution(int i_, double p1_, double p2_, double p3_ )
{
  parameter0 = i_;
  parameter1 = p1_;
  parameter2 = p2_;
  parameter3 = p3_;
} 


bool CatenaryParametersSolver::solve(double _xA, double _yA, double _xB, double _yB, float _l, float _dist, float _maxL, float _upper_bound)
{
  // Initial solution
  double x[4];
  x[0] = parameter0; // x0 
  x[1] = parameter1; // x0 
  x[2] = parameter2; // y0
  x[3] = parameter3; // a
  float dist_ = _dist;
  float maxL_ = _maxL;         
   // Build the problem.
   Problem problem;

  // Set up a cost funtion per point into the cloud
  CostFunction* cf1 = new ceres::AutoDiffCostFunction<CatenaryParameters, 4, 4>( new CatenaryParameters(_xA, _yA, _xB, _yB, _l, dist_, maxL_) );
  problem.AddResidualBlock(cf1, NULL, x);
  // if (parameter1 > 0.0)
  //   problem.SetParameterLowerBound(x, 0, 0.0001);

  problem.SetParameterLowerBound(x, 2, 0.01);
  // problem.SetParameterUpperBound(x, 2, _upper_bound);
	problem.SetParameterLowerBound(x, 3, 0.1);
  // problem.SetParameterUpperBound(x, 3, _upper_bound);

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = max_num_iterations;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  if(summary.message == "Initial residual and Jacobian evaluation failed.")
    printf("\t\t <<<< Failed in status");

  // Some debug information
  // std::cout << summary.BriefReport() << "\n";

  // Get the solution
  x0 = x[1]; y0 = x[2]; a = x[3]; 
  // std::cout <<"  x0: " << x0 << " , y0: " << y0 << " , a: "<< a << std::endl;
      
  return true; 
}

void CatenaryParametersSolver::getCatenaryParameters(double &_x0, double &_y0, double &_a)
{
  _x0 = x0;
  _y0 = y0;
  _a = a;
}



