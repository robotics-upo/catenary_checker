#ifndef POINT3D_CAT_CHECKER__
#define POINT3D_CAT_CHECKER__
#include <string>
#include <sstream>

#include "yaml-cpp/yaml.h"

struct Point3D {    //define points for 2d plane
  float x, y, z;

  Point3D(float _x = 0.0f, float _y = 0.0f, float _z = 0.0f):x(_x), y(_y), z(_z) {}

  Point2D(const YAML::Node &e) {
    x = e[0].as<float>();
    y = e[1].as<float>();
  }

  inline std::string toString() const {
    std::ostringstream oss;

    oss << "(" << x << ", " << y << ", " << z <<  ")" ;
    
    return oss.str();
  }

  inline void normalize (float new_norm = 1.0f) {
    float norm = sqrtf(x*x + y*y + z*z);
    x /= norm;
    y /= norm;
    z /= norm;
    if (new_norm != 1.0f ) {
        x *= new_norm;
        y *= new_norm;
        z *= new_norm;
    }
  }

};

inline YAML::Emitter& operator << (YAML::Emitter &out, const Point3D &p) {
  out << YAML::Flow;
  out << YAML::BeginSeq << p.x << p.y << p.z << YAML::EndSeq;

  return out;
}


#endif
