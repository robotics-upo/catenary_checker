#ifndef POINT2D_CAT_CHECKER__
#define POINT2D_CAT_CHECKER__
#include <string>
#include <sstream>

#include "yaml-cpp/yaml.h"

struct Point2D {    //define points for 2d plane
  float x, y;

  Point2D(float _x = 0.0f, float _y = 0.0f):x(_x), y(_y) {}

  Point2D(const YAML::Node &e) {
    x = e[0].as<float>();
    y = e[1].as<float>();
  }

  inline std::string toString() const {
    std::ostringstream oss;

    oss << "(" << x << ", " << y << ")";
    
    return oss.str();
  }

};

inline YAML::Emitter& operator << (YAML::Emitter &out, const Point2D &p) {
  out << YAML::Flow;
  out << YAML::BeginSeq << p.x << p.y << YAML::EndSeq;

  return out;
}


#endif
