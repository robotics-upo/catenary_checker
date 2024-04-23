#ifndef PARABLE_HPP__
#define PARABLE_HPP__

#include "catenary_checker/obstacle_2d.hpp"
#include <vector>
#include <QSplineSeries>

// Stores a parabola of the form: y = ax² + bx + c
class Parabola {
public:
  Parabola();

  Parabola(float a, float b, float c);

  Parabola(const Point2D &p1, const Point2D &p2, const Point2D &p3);

  std::string toString() const;

  QtCharts::QSplineSeries *toSeries(const std::string &name, float x0,
				    float x1, float spacing = 0.1f) const;

  float apply(float x) const;

  bool getParabola(const Point2D &p1, const Point2D &p2, const Point2D &p3);

  bool approximateParabola(const std::vector<Obstacle2D> &objects, Point2D &A,
			  Point2D &B, float min_y = 0.0);

  float getLength(float &x1, float &x2, float delta_t = 0.01) const;

  std::vector<Point2D> getPoints(float &x1, float &x2, float delta_t = 0.01) const;

  float _a, _b, _c;

  inline friend bool operator==(const Parabola &a, const Parabola &b)
  {
    return a._a == b._a && a._b == b._b && a._c == b._c;
  }
};

#endif
