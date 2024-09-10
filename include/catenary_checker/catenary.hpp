#ifndef CATENARY_HPP__
#define CATENARY_HPP__

#include <vector>
#include <catenary_checker/point_2d.hpp>
#include <QSplineSeries>

// Stores a Catenary of the form: y = ax² + bx + c
class Catenary {
public:
  Catenary();

  Catenary(double x0, double y0, double a);

  Catenary(const Point2D &p1, const Point2D &p2, const Point2D &p3);

  std::string toString() const;

  QtCharts::QSplineSeries *toSeries(const std::string &name, double x0,
				    double x1, double spacing = 0.1f) const;

  double apply(double x) const;

  bool approximateByLength(Point2D &A, Point2D &B, double length);

  double getLength(const double &x1,const double &x2) const;

  double getLengthApprox(const double &x1, const double &x2, double delta_t = 0.01) const;

  std::vector<Point2D> getPoints(double &x1, double &x2, double delta_t = 0.01) const;

  inline friend bool operator==(const Catenary &a, const Catenary &b)
  {
    return a._x0 == b._x0 && a._y0 == b._y0 && a._a == b._a;
  }

  double _x0, _y0, _a;
};

#endif
