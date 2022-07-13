#ifndef OBSTACLE_2D_HPP__
#define OBSTACLE_2D_HPP__

#include <vector>
#include <catenary_checker/point_2d.hpp>
#include <functional>
#include <QtCharts/QScatterSeries>

class Obstacle2D:public std::vector<Point2D>
{
public:
  Obstacle2D();

  Obstacle2D(const Obstacle2D &points);

  std::string toString() const;

  QtCharts::QScatterSeries *toSeries(const std::string &name = "obstacle",
				     float size = 3.0f, const QColor &color = QColor("red")) const;

  void add(const Obstacle2D &obstacle);

  void calculateConvexHull();
        
  bool intersects(std::function<float (float) > &func) const;
    
  std::vector<Point2D> convex_hull;

  static Obstacle2D rectangle(const Point2D &v1, const Point2D &v2, float spacing = 0.1f);

  static Obstacle2D randomObstacle(const Point2D &p, int n_points, float std_dev = 1.0f);
};

#endif
