#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
//#include <tf/transform_broadcaster.h>

#include <tf_conversions/tf_eigen.h>

#include <boost/foreach.hpp>

#include "simple_scan_matcher/scan_matcher.h"

#include <dynamic_reconfigure/server.h>
#include <simple_scan_matcher/SimpleScanMatcherReconfigureConfig.h>

using namespace simple_scan_matcher;

namespace
{
  typedef simple_scan_matcher::SimpleScanMatcherReconfigureConfig Conf;
  typedef dynamic_reconfigure::Server<Conf>                       ReconfServer;
}

class ScanMatcherNode
{
 public:

  ScanMatcherNode() : _use_max_range(false), _new_scan(false), _pose_stamped(false),
    _throttle(1), _kf_min_dist(0.25), _kf_min_ang(0.17),
    _base_frame("base_link"), _laser_frame("base_laser_link"), _pose_frame("world"), _nh("~")
  {
    ros::NodeHandle nh;
    _sub_scan = nh.subscribe("scan_in", 1, &ScanMatcherNode::laserScanCb, this);

    _dynreconf_srv.reset(new ReconfServer(ros::NodeHandle("simple_scan_matcher_dynreconf")));

    ReconfServer::CallbackType cb;
    cb = boost::bind(&ScanMatcherNode::drcb, this, _1, _2);

    _dynreconf_srv->setCallback(cb);

    _nh.param("throttle",    _throttle,    _throttle);
    _nh.param("min_dist",    _kf_min_dist, _kf_min_dist);
    _nh.param("min_ang",     _kf_min_ang,  _kf_min_ang);
    _nh.param("base_frame",  _base_frame,  _base_frame);
    _nh.param("laser_frame", _laser_frame, _laser_frame);
    _nh.param("pose_frame",  _pose_frame,  _pose_frame);

    _pub_pose = _nh.advertise<geometry_msgs::Pose2D>("pose2D", 1);

    if (_pub_pose_stamped)
      _pub_pose_stamped = _nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    _kf_min_dist *= _kf_min_dist;

    _getBaseToLaserTf();
  }

  ~ScanMatcherNode() { }

  void laserScanCb(sensor_msgs::LaserScanConstPtr scan_ptr)
  {
    if (scan_ptr == NULL)                      return;
    if (scan_ptr->header.seq % _throttle != 0) return;

    float max_range       = scan_ptr->range_max;
    float angle_min       = scan_ptr->angle_min;
    float angle_increment = scan_ptr->angle_increment;

    _scan.clear();

    if (_theta_map.empty() || _theta_map.size() < scan_ptr->ranges.size())
    {
      _theta_map.clear();

      for (int i=0; i<scan_ptr->ranges.size(); ++i)
        _theta_map.push_back( angle_min + float(i) * angle_increment );
    }

    for (int i=0; i<scan_ptr->ranges.size(); ++i)
    {
      float range = scan_ptr->ranges[i];
      float theta = _theta_map[i];

      if (!_use_max_range && range >= max_range)  continue;
      if (std::isnan(range) || std::isinf(range)) continue;

      Point2D p = Point2D::makePoint2DFromPolarCoordinates(range, theta);

      _scan.push_back(p);
    }

    _stamp = scan_ptr->header.stamp;
    _new_scan = true;
  }

  void process()
  {
    if (_scan.empty() || !_new_scan) return;

    // Init kf scan
    if (_kf_scan.empty())
    {
      ROS_DEBUG("Init. KF scan;");
      _kf_scan = _scan;
      return;
    }

    _sim_rel = _matcher.computeTransform(_kf_scan, _scan);

    if (_hasMovedEnough(_sim_rel))
    {
      ROS_DEBUG_STREAM("Update KF scan.");
      _updateKFScan();
    }

    ROS_DEBUG_STREAM("Estimated sim : \n\t x: " << _sim_rel.getX() <<
                    " y: " << _sim_rel.getY() << " theta: " << _sim_rel.getTheta());

    _new_scan = false;
  }

  void publish()
  {
    Similitude sim_pub = _sim_cumu * _sim_rel;

    // get pose (relative to last kf) in laser frame
    tf::Transform pose_lf = _XYThetaToTf(sim_pub.getX(), sim_pub.getY(), sim_pub.getTheta());

    // get pose (relative to last kf) in base frame
    tf::Transform pose_bf = /*_laser_to_base **/ pose_lf;

    geometry_msgs::Pose2D pose2D;

    pose2D.x = pose_bf.getOrigin().getX();
    pose2D.y = pose_bf.getOrigin().getY();

    pose2D.theta = tf::getYaw(pose_bf.getRotation());

    _pub_pose.publish(pose2D);

    if (_pose_stamped)
    {
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.pose.position.x = pose2D.x;
      poseStamped.pose.position.y = pose2D.y;
      poseStamped.pose.position.z = 0;

      tf::quaternionTFToMsg(pose_bf.getRotation(), poseStamped.pose.orientation);

      poseStamped.header.stamp    = _stamp;
      poseStamped.header.frame_id = _pose_frame;

      _pub_pose_stamped.publish(poseStamped);
    }
  }

private:

  bool _use_max_range, _new_scan, _pose_stamped;

  int _throttle;

  double _kf_min_dist, _kf_min_ang;

  std::string _base_frame, _laser_frame, _pose_frame;

  std::vector<float> _theta_map;

  ros::Time _stamp;
  Scan _kf_scan, _scan;

  Similitude _sim_rel, _sim_cumu;

  ros::NodeHandle _nh;

  ros::Subscriber _sub_scan;
  ros::Publisher  _pub_pose;
  ros::Publisher  _pub_pose_stamped;

  tf::TransformListener    _listener;
//  tf::TransformBroadcaster _broadcaster;

  tf::Transform _base_to_laser;
  tf::Transform _laser_to_base;

  boost::shared_ptr<ReconfServer> _dynreconf_srv;

  ScanMatcher _matcher;

  void drcb(Conf &config, uint32_t level)
  {
    _use_max_range = config.use_max_range;
    _kf_min_dist   = config.min_dist;
    _kf_min_ang    = config.min_ang;
    _throttle      = config.throttle;

    ROS_INFO_STREAM("Reconf");
  }

  bool _hasMovedEnough(const Similitude& sim)
  {
    if ((sim.getX()*sim.getX() + sim.getY()*sim.getY()) > _kf_min_dist) return true;

    if (std::fabs(sim.getTheta()) > _kf_min_ang) return true;

    return false;
  }

  void _updateKFScan()
  {
    _kf_scan = _scan;
    _sim_cumu *= _sim_rel;
  }

  bool _getBaseToLaserTf()
  {
    tf::StampedTransform base_to_laser;
    base_to_laser.setIdentity();

    bool got_tf = false;
    try
    {
      _listener.waitForTransform(
            _base_frame, _laser_frame, ros::Time(0), ros::Duration(2.0));
      _listener.lookupTransform (
            _base_frame, _laser_frame, ros::Time(0), base_to_laser);

      got_tf = true;
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("Could not get initial transform from base to laser frame, %s", ex.what());
    }

    _base_to_laser = base_to_laser;
    _laser_to_base = base_to_laser.inverse();

    return got_tf;
  }

  tf::Transform _XYThetaToTf(double x, double y, double theta)
  {
    tf::Transform t;
    t.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    t.setRotation(q);

    return t;
  }

  geometry_msgs::Pose _Pose2DToPose(const geometry_msgs::Pose2D pose2D)
  {
    geometry_msgs::Pose pose;

    // use tf-pkg to convert angles
    tf::Quaternion frame_quat;

    // transform angle from euler-angle to quaternion representation
    frame_quat = tf::createQuaternionFromYaw(pose2D.theta);

    // set position
    pose.position.x = pose2D.x;
    pose.position.y = pose2D.y;
    pose.position.z = 0.0;

    // set quaternion
    pose.orientation.x = frame_quat.x();
    pose.orientation.y = frame_quat.y();
    pose.orientation.z = frame_quat.z();
    pose.orientation.w = frame_quat.w();

    return pose;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_scan_matcher");

  ScanMatcherNode scan_matcher;

  ros::Rate rate(50);

  while (ros::ok())
  {
    scan_matcher.process();

    scan_matcher.publish();

    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}
