#ifndef POINT2D_CAT_CHECKER__
#define POINT2D_CAT_CHECKER__
#include <string>
#include <sstream>

struct Point2D {    //define points for 2d plane
  float x, y;

  Point2D(float _x = 0.0f, float _y = 0.0f):x(_x), y(_y) {}

  inline std::string toString() {
    std::ostringstream oss;

    oss << "(" << x << ", " << y << ")";
    
    return oss.str();
  }
};

#endif
