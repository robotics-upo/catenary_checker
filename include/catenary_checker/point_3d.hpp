#ifndef POINT3D_CAT_CHECKER__
#define POINT3D_CAT_CHECKER__
#include <string>
#include <sstream>
#include <math.h>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include "yaml-cpp/yaml.h"

struct Point3D {    //define points for 2d plane
  float x, y, z;

  Point3D(float _x = 0.0f, float _y = 0.0f, float _z = 0.0f):x(_x), y(_y), z(_z) {}

  Point3D(const YAML::Node &e) {
    x = e[0].as<float>();
    y = e[1].as<float>();
    z = e[2].as<float>();
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

    inline pcl::PointXYZ toPCL() const {
        pcl::PointXYZ p;
        p.x = x; p.y = y; p.z = z;

        return p;
    }

    inline static Point3D fromPCL(const pcl::PointXYZ &p) {
      Point3D ret;
      ret.x = p.x; ret.y = p.y; ret.z = p.z;

      return ret;
    }
    
};

//! addition operator (adds each coordinate)
inline Point3D operator+(const Point3D &left, const Point3D &right) {
    return Point3D(left.x + right.x, left.y + right.y, left.z + right.z);
}

//! substraction operator (substracts each coordinate)

inline Point3D operator-(const Point3D &left, const Point3D &right) {
    return Point3D(left.x - right.x, left.y - right.y, left.z - right.z);
}

//! Opposite
inline Point3D operator-(const Point3D &p) {
    return Point3D(-p.x, -p.y, -p.z);
}

inline YAML::Emitter& operator << (YAML::Emitter &out, const Point3D &p) {
  out << YAML::Flow;
  out << YAML::BeginSeq << p.x << p.y << p.z << YAML::EndSeq;

  return out;
}

#endif
