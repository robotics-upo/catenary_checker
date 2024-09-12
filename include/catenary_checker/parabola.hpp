#ifndef PARABLE_HPP__
#define PARABLE_HPP__

#include "catenary_checker/obstacle_2d.hpp"
#include <vector>
#include <QSplineSeries>
#include "catenary_checker/scenario.hpp"

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

  //! @brief Uses the algorithm specified in our paper to get a valid
  //! @brief parable given a vector of 2D objects in a plane from A to B.
  bool approximateParabola(const Scenario &objects, const Point2D &A,
                          const Point2D &B, float min_y = 0.0);

  float getLength(float &x1, float &x2) const;

  float getLengthApprox(float &x1, float &x2, float delta_t = 0.01) const;

  std::vector<Point2D> getPoints(float &x1, float &x2, float delta_t = 0.01) const;

  float _a, _b, _c;

  inline friend bool operator==(const Parabola &a, const Parabola &b)
  {
    return a._a == b._a && a._b == b._b && a._c == b._c;
  }

  inline void reset() {_a = _b = _c = 0.0f;}

private:
  bool recursiveApproximateParabola(const Scenario &objects, const Point2D &A,
                                    const Point2D &B, float min_y = 0.0);
};

#endif
