#ifndef PARABLE_HPP__
#define PARABLE_HPP__

#include "catenary_checker/obstacle_2d.hpp"
#include <vector>
#include <QSplineSeries>

// Stores a parable of the form: y = ax² + bx + c
class Parable {
public:
  Parable();

  Parable(float a, float b, float c);

  Parable(const Point2D &p1, const Point2D &p2, const Point2D &p3);

  std::string toString() const;

  QtCharts::QSplineSeries *toSeries(const std::string &name, float x0,
				    float x1, float spacing = 0.1f) const;

  float apply(float x) const;

  bool getParable(const Point2D &p1, const Point2D &p2, const Point2D &p3);

  bool approximateParable(const std::vector<Obstacle2D> &objects, Point2D &A,
			  Point2D &B, float min_y = 0.0);

  float _a, _b, _c;

  inline friend bool operator==(const Parable &a, const Parable &b)
  {
    return a._a == b._a && a._b == b._b && a._c == b._c;
  }
};

#endif
