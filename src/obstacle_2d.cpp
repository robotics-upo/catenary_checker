#include "catenary_checker/obstacle_2d.hpp"
#include "catenary_checker/convex_hull.hpp"
#include <random>
#include <sstream>
#include <ostream>
#include <fstream>
#include <iostream>

using namespace std;

Obstacle2D::Obstacle2D():std::vector<Point2D>() {}

Obstacle2D::Obstacle2D(const Obstacle2D &points):std::vector<Point2D>(points) {}

void Obstacle2D::calculateConvexHull() {
  if (size() > 3) {
    convex_hull = findConvexHull(*this);
  }
  else {
    convex_hull.clear();
    for (auto &x:*this) {
      convex_hull.push_back(x);
    }
  }
}

bool Obstacle2D::intersects(std::function<float (float) > &func, double x_min, double x_max) const {
  bool exist_lower = false;
  bool exist_up = false;
  bool intersects = false;
  for (size_t p = 0; p < size() && !intersects;p++) {
    if (at(p).x < x_min || at(p).x > x_max) {
      continue; // We only take into account the obstacles between the desired Xs 
    }

    float fy = func(at(p).x);
    exist_up |= fy < at(p).y;
    exist_lower |= fy > at(p).y;
    intersects = exist_lower && exist_up;
  }
  return intersects;
}

void Obstacle2D::add(const Obstacle2D &obstacle) {
  for (auto &x:obstacle) {
    push_back(x);
  }
}

Obstacle2D Obstacle2D::rectangle(const Point2D &v1, const Point2D &v2, float spacing)
{
  Obstacle2D ret;

  spacing = fabsf(spacing);
  spacing = std::max(spacing, 1e-6f);
  
  float min_x = std::min(v1.x, v2.x);
  float max_x = std::max(v1.x, v2.x);
  Point2D p;
  for (float x = min_x; x < max_x; x += spacing)
  {
    float min_y = std::min(v1.y, v2.y);
    float max_y = std::max(v1.y, v2.y);
    for (float y = min_y; y < max_y; y += spacing)
    {
      p.x = x;
      p.y = y;
      ret.push_back(p);
    }
  }
      
  return ret;
}


Obstacle2D Obstacle2D::randomObstacle(const Point2D &p, int n_points, float std_dev)
{
  Obstacle2D ret;
  std::random_device rd;
  std::mt19937 mt(rd());
  std::normal_distribution<float> dist_x(p.x, std_dev);
  std::normal_distribution<float> dist_y(p.y, std_dev);
  
  Point2D np;
  for (int i = 0; i < n_points; i++) {
    np.x = dist_x(mt);
    np.y = dist_y(mt);
    ret.push_back(np);
  }

  return ret;
}

std::string Obstacle2D::toString() const {
  std::ostringstream oss;

  oss << "Obstacle2D. n_points: " << size() << std::endl;
  
  return oss.str();
}

using namespace QtCharts;

QScatterSeries *Obstacle2D::toSeries(const std::string &name, float size,
				     const QColor &color) const {
  QScatterSeries *ret = new QScatterSeries();
  ret->setName(QString::fromStdString(name));
  ret->setMarkerShape(QScatterSeries::MarkerShapeRectangle);
  ret->setMarkerSize(size);
  ret->setColor(color);

  for (auto &p:*this) {
    ret->append(p.x, p.y);
  }
  
  return ret;
}
  
// Exporter to YAML
using namespace YAML;
Emitter &operator << (Emitter &out, const Obstacle2D &o) {
  out << YAML::BeginSeq;
  for (const auto &p:o) {

    out << p;
  }
  out << YAML::EndSeq;
  return out;
}

// Get an obstacle from node
void Obstacle2D::fromYAML(const YAML::Node &n) {
  for (auto &x:n) {
    Point2D p(x);

    push_back(p);
  }
}

void Obstacle2D::simplify(double min_dist) {

  float min_x = 1e20;
  float min_y = 1e20;
  float max_x = -1e20;
  float max_y = -1e20;
  for (auto &x:*this) {
    min_x = std::min(min_x, x.x);
    max_x = std::max(max_x, x.x);
    min_y = std::min(min_y, x.y);
    max_y = std::max(max_y, x.y);
  }

  int rows = (max_y - min_y) / min_dist + 1; 
  int cols = (max_x - min_x) / min_dist + 1;

  std::vector<std::vector <bool> > occupied(false);

  occupied.resize(rows);
  for (auto &x:occupied) {
    x.resize(cols);
    for (int i = 0; i < cols; i++) {
      x[i] = false;
    }
  }

  std::vector<int> to_be_cleared;
  to_be_cleared.reserve(size());
  for (int curr = 0; curr < size(); curr++) {
    int j = (at(curr).x - min_x) / min_dist;
    int i = (at(curr).y - min_y) / min_dist;

    if (!occupied[i][j])
      occupied[i][j] = true;
    else
      to_be_cleared.push_back(curr);
  }

  for (int i = to_be_cleared.size() - 1; i >= 0; i--) {
    auto x = begin();
    x += to_be_cleared[i];
    
    this->erase(x);
  }
}
