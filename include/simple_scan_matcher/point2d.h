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

    typedef Eigen::Vector3d                    PointMat;
    typedef Eigen::aligned_allocator<PointMat> PointMatAlloc;
    typedef Eigen::Matrix3d                    SimMat;

   public:

    Point2D() { _point.setZero(); }

    Point2D(double x, double y)
    {
      _point(0) = x;
      _point(1) = y;
      _point(2) = 1;
    }

    Point2D(PointMat& point)
    {
      _point = point;
    }

    Point2D(const PointMat& point)
    {
      _point = point;
    }

    ~Point2D() { }

    double getX() const { return _point(0); }
    double getY() const { return _point(1); }

    double setX(double x) { _point(0) = x; }
    double setY(double y) { _point(1) = y; }

    static Point2D makePoint2DFromPolarCoordinates(double range, double theta)
    {
      return Point2D(std::cos(theta) * range, std::sin(theta) * range);
    }

    inline Point2D transform(const SimMat& m) const
    {
      return Point2D(m * _point);
    }

    Point2D& operator+=(const Eigen::Vector3d& v)
    {
      _point += v;

      return *this;
    }

  private:

    PointMat _point;

  };

  typedef std::vector<Point2D, Point2D::PointMatAlloc> Scan;

  typedef std::pair<uint, uint>       Correspondence;
  typedef std::vector<Correspondence> Correspondences;

} //namespace simple_scan_matcher

inline std::ostream& operator<<(std::ostream& out, const simple_scan_matcher::Point2D& p)
{
  return out << " (" << p.getX() << "," << p.getY() << ")";
}

inline std::ostream& operator<<(std::ostream& out, const simple_scan_matcher::Scan& s)
{
  for (size_t i=0; i<s.size(); ++i)
    out << s[i];

   return out;
}

#endif //SIMPLE_SCAN_MATCHER_POINT2D_H
