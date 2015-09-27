#ifndef SIMPLE_SCAN_MATCHER_POINT2D_H
#define SIMPLE_SCAN_MATCHER_POINT2D_H

#include <cmath>
#include <vector>

#include "simple_scan_matcher/similitude.h"

namespace simple_scan_matcher
{
  class Point2D
  {
   public:

    Point2D() : _x(0), _y(0) { }

    Point2D(double x, double y)
    {
      _x = x;
      _y = y;
    }

    ~Point2D() { }

    double getX() const { return _x; }
    double getY() const { return _y; }

    double setX(double x) { _x = x; }
    double setY(double y) { _y = y; }

    static Point2D makePoint2DFromPolarCoordinates(double range, double theta)
    {
      return Point2D(std::cos(theta) * range, std::sin(theta) * range);
    }

  private:

    double _x, _y;

  };

  typedef std::vector<simple_scan_matcher::Point2D> Scan;

} //namespace simple_scan_matcher



#endif //SIMPLE_SCAN_MATCHER_POINT2D_H
