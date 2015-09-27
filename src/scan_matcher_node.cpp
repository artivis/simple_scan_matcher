#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>

#include <boost/foreach.hpp>

#include "simple_scan_matcher/scan_matcher.h"

using namespace simple_scan_matcher;

class ScanMatcherNode
{
 public:

  ScanMatcherNode() : _use_max_range(false), _throttle(1), _nh("")
  {
    _sub = _nh.subscribe("scan_in", 1, &ScanMatcherNode::laserScanCb, this);

    //_pub = _nh.advertise("relative_transform", 1, false);

  }

  ~ScanMatcherNode() { }

  void laserScanCb(sensor_msgs::LaserScanConstPtr scan_ptr)
  {
    if (scan_ptr == NULL)                      return;
    if (scan_ptr->header.seq % _throttle != 0) return;

    float max_range       = scan_ptr->range_max;
    float angle_min       = scan_ptr->angle_min;
    float angle_increment = scan_ptr->angle_increment;

    _prev_scan = _scan;

    _scan.clear();

    for (int i=0; i<scan_ptr->ranges.size(); ++i)
    {
      float range = scan_ptr->ranges[i];
      float theta = angle_min + float(i) * angle_increment;

      if (!_use_max_range && range >= max_range)  continue;
      if (std::isnan(range) || std::isinf(range)) continue;

      Point2D p = Point2D::makePoint2DFromPolarCoordinates(range, theta);

      _scan.push_back(p);
    }
  }

  void process()
  {
    if (_prev_scan.empty() || _scan.empty()) return;

    _matcher.computeTransform(_prev_scan, _scan);
  }

private:

  bool _use_max_range;

  int _throttle;

  Scan _prev_scan, _scan;

  ros::NodeHandle _nh;

  ros::Subscriber _sub;
  ros::Publisher  _pub;

  ScanMatcher _matcher;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_scan_matcher");

  ScanMatcherNode scan_matcher;

  ros::Rate rate(50);

  while (ros::ok())
  {
    scan_matcher.process();

    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}
