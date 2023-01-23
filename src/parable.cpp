#include "catenary_checker/parable.hpp"
#include <limits>
#include <iostream>
#include <sstream>
#include <cmath>

Parable::Parable() { _a = _b = _c = 0.0f; }

Parable::Parable(float a, float b, float c) {
    _a = a;
    _b = b;
    _c = c;
}

Parable::Parable(const Point2D &p1, const Point2D &p2, const Point2D &p3) {
    getParable(p1, p2, p3);
}

float Parable::apply(float x) const { 
    return _a * x * x + _b * x + _c;
}

bool Parable::getParable(const Point2D &p1, const Point2D &p2, const Point2D &p3) 
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

bool Parable::approximateParable(const std::vector<Obstacle2D> &objects, Point2D &A,
				 Point2D &B, float min_y) 
{
  Obstacle2D artificial_obs;
  artificial_obs.push_back(A);
  artificial_obs.push_back(B);

  Parable back(*this);

  if (_a == 0.0 && _b == 0.0 && _c == 0.0) {
    _b = (A.y - B.y)/(A.x - B.x);
    _c = A.y - _b * A.x;
    std::cout << "Primera parabola. Parametros: " << toString() << "\n";
  }

  std::function<float(float)> f = std::bind(&Parable::apply, this, std::placeholders::_1);
  for (auto &x:objects) {
    if (x.intersects(f)) {
      std::cout << "Adding obstacle: " << x.toString() << std::endl;
      artificial_obs.add(x);
    }
  }

  std::cout << "Parable::approximateParable -->  adding intersecting obstacles: number: "
	    << artificial_obs.size() << std::endl;

  if (artificial_obs.size() < 3 ) {
    std::cout << "Parabola aproximada." << std::endl;
    return true;
  }

  artificial_obs.calculateConvexHull();

  float min_x = std::min(A.x, B.x);
  float max_x = std::max(A.x, B.x);

  for (auto &x:artificial_obs.convex_hull) {
    if (x.y < A.y || x.x < min_x || x.x > max_x) {
      std::cout << "Parable::approximateParable ";  
      std::cout << "Convex Hull failure: not restricted to the limits" << std::endl;
      std::cout << "Conflicting point: " << x.toString() << std::endl;
      return false;
    }
  }

  float max_a = -std::numeric_limits<float>::infinity();
  float best_b = 0.0;
  float best_c = 0.0;

  for (auto &x:artificial_obs.convex_hull) {
    if (!getParable(A, x, B)) {
      max_a = _a;
      best_b = _b;
      best_c = _c;
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
    std::cout << "Error: Parable::approximateParable the parable passes below: "
	      << min_y << std::endl;
    return false;
  }

  std::cout << "Parable::approximateParable. New params: " << toString() << std::endl;
  if (back == *this) {
    std::cout << "Error: Parable::approximateParable: Detected same parable --> fail";
    return false;
  }

  return approximateParable(objects, A, B, min_y);
}

std::string Parable::toString() const {
    std::ostringstream oss;

    oss << "Parable parameters: (" << _a << ", " << _b ;
    oss << ", " << _c << ")\n";


    return oss.str();
}

std::vector<Point2D> Parable::getPoints(float &x1, float &x2, float delta_t) const {
  std::vector<Point2D> ret_val;
  for (float x = x1; x <= x2; x+=delta_t) {
    Point2D p(x, apply(x));
    ret_val.push_back(p);
  }

  return ret_val;
}
 
float Parable::getLength(float &x1, float &x2, float delta_t) const {
  // We have to integrate: integral(x1,x2) of: sqrt(1+df/dx^2)dx
  // sqrt(1+(2ax+b)^2) = sqrt(1+4a²x²+b²+4abx)dx
  // This integral has no primitive --> should be approximated
  float ret_val=0.0f;

  float a_2 = _a * _a * 4.0f;
  float b_2 = _b * _b;
  float a_b_4 = _a * _b * 4.0f;

  for (float x = x1; x <= x2; x += delta_t) {
    ret_val += std::sqrt(1 + b_2 + a_2 * x * x + a_b_4 * x);
  }
  ret_val *= delta_t;

  return ret_val;

}



/// Using Qt Charts for representation
using namespace QtCharts;

QSplineSeries *Parable::toSeries(const std::string &name,
				 float x0, float x1, float spacing) const {
  QSplineSeries *ret = new QSplineSeries();
  ret->setName(QString::fromStdString(name));
  ret->setColor(QColor("red"));

  for (float x = x0; x <= x1; x += spacing) {
    ret->append(x, apply(x));
  }
  
  return ret;



}


