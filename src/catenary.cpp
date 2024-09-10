#include "catenary_checker/catenary.hpp"
#include <limits>
#include <iostream>
#include <sstream>
#include <cmath>

// Ceres libraries
#include "ceres/ceres.h"
#include "glog/logging.h"

// Workspace
using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::LossFunction;
using ceres::AutoDiffCostFunction;

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
        {}

        template <typename T>
        bool operator()(const T* P_, T* R_) const 
        {   
            R_[0] = T{100.0}*(P_[2] * cosh((xA - P_[0])/P_[2]) + P_[1]  - yA);
            R_[1] = T{100.0}*(P_[2] * cosh((xB - P_[0])/P_[2]) + P_[1]  - yB);
            T La = P_[2]*tanh((xA - P_[0])/P_[2])*sqrt(sinh((xA - P_[0])/P_[2])*sinh((xA - P_[0])/P_[2]) + T{1.0});
            T Lb = P_[2]*tanh((xB - P_[0])/P_[2])*sqrt(sinh((xB - P_[0])/P_[2])*sinh((xB - P_[0])/P_[2]) + T{1.0});
            T len = Lb - La;
            R_[2] = (len - L);
            // std::cout << "          Length : L = " << L << " , len = " << len << std::endl;   
            return true;
        }

    private:
        // Point to be evaluated
        double xA, yA, xB, yB, L;
};

Catenary::Catenary() { _x0 = _y0 = _a = 0.0f; }

Catenary::Catenary(double x0, double y0, double a) {
    _a = a;
    _x0 = x0;
    _y0 = y0;
}

double Catenary::apply(double x) const { 
    return (_a * cosh((x - _x0)/_a) + ( _y0));
}

std::string Catenary::toString() const {
    std::ostringstream oss;

    oss << "Catenary parameters: (" << _x0 << ", " << _y0 ;
    oss << ", " << _a << ")\n";


    return oss.str();
}

std::vector<Point2D> Catenary::getPoints(double &x1, double &x2, double delta_t) const {
  std::vector<Point2D> ret_val;
  // std::cout << "x1 = " << x1 << " , x2 = " << x2 << std::endl;
  for (double x = x1; x <= x2; x+=delta_t) {
    Point2D p(x, apply(x));
    // std::cout << "p.x = " << p.x << " , p.y = " << p.y << std::endl;
    ret_val.push_back(p);
  }

  return ret_val;
}
 
double Catenary::getLengthApprox(const double &x1, const double &x2, double delta_t) const {
  // We have to integrate: integral(x1,x2) of: sqrt(1+df/dx^2)dx
  // sqrt(1+(2ax+b)^2) = sqrt(1+4a²x²+b²+4abx)dx
  // This integral has no primitive --> should be approximated
  double ret_val = 0.0;
  double y0 = apply(x1);

  for (double x = x1 + delta_t; x <= x2; x += delta_t) {
    double y1 = apply(x);
    ret_val += std::sqrt(delta_t*delta_t + (y1 - y0)*(y1 - y0) );
    y0 = y1;
  }

    return ret_val;
}

double Catenary::getLength(const double &x1,const double &x2) const {

    double La = _a*tanh((x1 - _x0)/_a)*sqrt(sinh((x1 - _x0)/_a)*sinh((x1 - _x0)/_a) + 1.0);
    double Lb = _a*tanh((x2 - _x0)/_a)*sqrt(sinh((x2 - _x0)/_a)*sinh((x2 - _x0)/_a) + 1.0);
    return (Lb - La);
}

bool Catenary::approximateByLength(Point2D &A, Point2D &B, double length) {
    double cat[3];
    int max_num_iterations = 5000;

    cat[0] = 0.5; // x0 
    cat[1] = -1.0; // y0
    cat[2] = 1.5; // a      
                
    // Build the problem.
    Problem problem_catenary;

    // Set up a cost funtion per point into the cloud
    CostFunction* cf_catenary = new ceres::AutoDiffCostFunction<CatenaryParameters, 3, 3>( new CatenaryParameters(A.x, A.y, B.x, B.y, length) );
    problem_catenary.AddResidualBlock(cf_catenary, NULL, cat);
    problem_catenary.SetParameterLowerBound(cat, 0, 0.0);
    problem_catenary.SetParameterLowerBound(cat, 2, 0.1);
    // Run the solver!
    Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = max_num_iterations;
    Solver::Summary summary1;
    Solve(options, &problem_catenary, &summary1);

    _x0 = cat[0]; _y0 = cat[1]; _a = cat[2];
    
    return true;
}

/// Using Qt Charts for representation
using namespace QtCharts;

QSplineSeries *Catenary::toSeries(const std::string &name,
				 double x0, double x1, double spacing) const {
  QSplineSeries *ret = new QSplineSeries();
  ret->setName(QString::fromStdString(name));
  ret->setColor(QColor("red"));

  for (double x = x0; x <= x1; x += spacing) {
    ret->append(x, apply(x));
  }
  
  return ret;
}


