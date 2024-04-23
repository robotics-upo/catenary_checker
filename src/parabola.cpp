#include "catenary_checker/parabola.hpp"
#include <limits>
#include <iostream>
#include <sstream>
#include <cmath>

Parabola::Parabola() { _a = _b = _c = 0.0f; }

Parabola::Parabola(float a, float b, float c) {
    _a = a;
    _b = b;
    _c = c;
}

Parabola::Parabola(const Point2D &p1, const Point2D &p2, const Point2D &p3) {
    getParabola(p1, p2, p3);
}

float Parabola::apply(float x) const { 
    return _a * x * x + _b * x + _c;
}

bool Parabola::getParabola(const Point2D &p1, const Point2D &p2, const Point2D &p3) 
{
    if (p1.x == p2.x || p2.x == p3.x || p1.x == p3.x)
        return false;

    float x1 = p1.x;
    float x2 = p2.x;
    float x3 = p3.x;
    float y1 = p1.y;
    float y2 = p2.y;
    float y3 = p3.y;

    _a = ((x1-x2)*(y1-y3)-(y1-y2)*(x1-x3)) /
        ((x1-x3)*(x1-x2)*(x3-x2));
    _b = (y1-y2-_a*(x1*x1 - x2*x2))/(x1 - x2);
    _c = y3 - _a*(x3*x3) - _b*x3;
    
    return true;
}

bool Parabola::approximateParabola(const std::vector<Obstacle2D> &objects, Point2D &A,
				 Point2D &B, float min_y) 
{
  // std::cout << "approximateParabola    _a: " << _a << " , _b: " << _b <<  " , _c: " << _c <<std::endl;
  Obstacle2D artificial_obs;
  std::vector<Obstacle2D> nonIntersection;
  // artificial_obs.push_back(A);
  // artificial_obs.push_back(B);

  Parabola back(*this);

  if (_a == 0.0 && _b == 0.0 && _c == 0.0) {
    _b = (A.y - B.y)/(A.x - B.x);
    _c = A.y - _b * A.x;
    // std::cout << "Primera parabola. Parametros: " << toString() << "\n";
  }

  std::function<float(float)> f = std::bind(&Parabola::apply, this, std::placeholders::_1);
  int flag_ = 0;
  for (auto &x:objects) {
    if (x.intersects(f)) {
      // std::cout << "Adding obstacle: " << x.toString() << std::endl;
      // std::cout << "obstacle added: x=" << x[flag_].x << " y=" << x[flag_].y <<std::endl;
      artificial_obs.add(x);
    }
    else{
      nonIntersection.push_back(x);
    }
    flag_++;
  }

  // std::cout << "Parabola::approximateParabola -->  adding intersecting obstacles: number: " << artificial_obs.size() << std::endl;

  if (artificial_obs.size() < 1 ) {
    // std::cout << "Parabola aproximada." << std::endl;
    return true;
  }

  //  std::cout << "Before calculateConvexHull." << std::endl;

  artificial_obs.calculateConvexHull();

  //  std::cout << "After calculateConvexHull." << std::endl;


  float min_x = std::min(A.x, B.x);
  float max_x = std::max(A.x, B.x);

  for (auto &x:artificial_obs.convex_hull) {
    if (x.y < A.y || x.x < min_x || x.x > max_x) {
      // std::cout << "Parabola::approximateParabola ";  
      // std::cout << "Convex Hull failure: not restricted to the limits" << std::endl;
      // std::cout << "Conflicting point: " << x.toString() << std::endl;
      return false;
    }
  }

  float max_a = -std::numeric_limits<float>::infinity();
  float best_b = 0.0;
  float best_c = 0.0;

  for (auto &x:artificial_obs.convex_hull) {
    if (getParabola(A, x, B)) {
      if (max_a < _a){
      //  std::cout << "_a: " << _a << " , _b: " << _b <<  " , _c: " << _c <<std::endl;
        max_a = _a;
        best_b = _b;
        best_c = _c;
      }
    }
  }
    
  _a = max_a;
  _b = best_b;
  _c = best_c;

  if (_a < 0.0) {
    return false;
  }

  float lx = - _b/(2*_a);
  float ly = apply(lx);
  if (ly < min_y) {
    // std::cout << "Error: Parabola::approximateParabola the parable passes below: "  << min_y << std::endl;
    return false;
  }

  // std::cout << "Parabola::approximateParabola. New params: " << toString() << std::endl;
  if (back == *this) {
    // std::cout << "Error: Parabola::approximateParabola: Detected same parable --> fail";
    return false;
  }

  // return approximateParabola(objects, A, B, min_y);
  return approximateParabola(nonIntersection, A, B, min_y);
}

std::string Parabola::toString() const {
    std::ostringstream oss;

    oss << "Parabola parameters: (" << _a << ", " << _b ;
    oss << ", " << _c << ")\n";


    return oss.str();
}

std::vector<Point2D> Parabola::getPoints(float &x1, float &x2, float delta_t) const {
  std::vector<Point2D> ret_val;
  // std::cout << "x1 = " << x1 << " , x2 = " << x2 << std::endl;
  for (float x = x1; x <= x2; x+=delta_t) {
    Point2D p(x, apply(x));
    // std::cout << "p.x = " << p.x << " , p.y = " << p.y << std::endl;
    ret_val.push_back(p);
  }

  return ret_val;
}
 
float Parabola::getLength(float &x1, float &x2, float delta_t) const {
  // We have to integrate: integral(x1,x2) of: sqrt(1+df/dx^2)dx
  // sqrt(1+(2ax+b)^2) = sqrt(1+4a²x²+b²+4abx)dx
  // This integral has no primitive --> should be approximated
  float ret_val=0.0f;
  float ret_val1=0.0f; 
  float ret_val2=0.0f; 

  float a_2 = _a * _a * 4.0f;
  float b_2 = _b * _b;
  float a_b_4 = _a * _b * 4.0f;

  for (float x = x1; x <= x2; x += delta_t) {
    ret_val += std::sqrt(1 + b_2 + a_2 * x * x + a_b_4 * x);
  }
  ret_val *= delta_t;
    // ret_val2 = std::sqrt(1 + b_2 + a_2 * x2 * x2 + a_b_4 * x2);
    // ret_val1 = std::sqrt(1 + b_2 + a_2 * x1 * x1 + a_b_4 * x1);
    // ret_val = ret_val2 - ret_val1;

  return ret_val;

}



/// Using Qt Charts for representation
using namespace QtCharts;

QSplineSeries *Parabola::toSeries(const std::string &name,
				 float x0, float x1, float spacing) const {
  QSplineSeries *ret = new QSplineSeries();
  ret->setName(QString::fromStdString(name));
  ret->setColor(QColor("red"));

  for (float x = x0; x <= x1; x += spacing) {
    ret->append(x, apply(x));
  }
  
  return ret;



}


