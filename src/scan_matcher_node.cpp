#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
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

  #define ROS_RED_STREAM(x)   ROS_INFO_STREAM("\033[1;31m" << x << "\033[0m")
}

class ScanMatcherNode
{
 public:

  ScanMatcherNode() : _use_max_range(false), _new_scan(false), _pose_stamped(false),
    _throttle(1), _kf_min_dist(0.15), _kf_min_ang(0.10), //0.05  0.15
    _base_frame("base_link"), _laser_frame("base_laser_link"), _pose_frame("world"), _nh("~")
  {
    ros::NodeHandle nh;
    _sub_scan = nh.subscribe("scan_in", 1, &ScanMatcherNode::laserScanCb, this);

    _dynreconf_srv.reset(new ReconfServer(ros::NodeHandle("simple_scan_matcher_dynreconf")));

    ReconfServer::CallbackType cb;
    cb = boost::bind(&ScanMatcherNode::drcb, this, _1, _2);

    _dynreconf_srv->setCallback(cb);

    _nh.param("throttle",     _throttle,     _throttle);
    _nh.param("min_dist",     _kf_min_dist,  _kf_min_dist);
    _nh.param("min_ang",      _kf_min_ang,   _kf_min_ang);
    _nh.param("base_frame",   _base_frame,   _base_frame);
    _nh.param("laser_frame",  _laser_frame,  _laser_frame);
    _nh.param("pose_frame",   _pose_frame,   _pose_frame);
    _nh.param("pose_stamped", _pose_stamped, _pose_stamped);

    _pub_pose = _nh.advertise<geometry_msgs::Pose2D>("pose2D", 1);

    if (_pose_stamped)
      _pub_pose_stamped = _nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    _pub_scan = _nh.advertise<visualization_msgs::Marker>("kf_scan", 1);

    _kf_min_dist/* *= _kf_min_dist*/;

    _getBaseToLaserTf();

    _getInitialPoseTf(_sim_cumu);

    ROS_INFO_STREAM("Initial Sim : \n\t x: " << _sim_cumu.getX() <<
                    " y: " << _sim_cumu.getY() << " theta: " << _sim_cumu.getTheta()
                    << " scale: " << _sim_cumu.getScale());

    ROS_DEBUG_STREAM("Initial T : \n" << _sim_cumu.getTransformationMat());
  }

  ~ScanMatcherNode() { }

  void laserScanCb(sensor_msgs::LaserScanConstPtr scan_ptr)
  {
    if (scan_ptr == NULL)                      return;
    if (scan_ptr->header.seq % _throttle != 0) return;

    double max_range       = (double)scan_ptr->range_max;
    double angle_min       = (double)scan_ptr->angle_min;
    double angle_increment = (double)scan_ptr->angle_increment;

    _scan.clear();

    if (_theta_map.empty() || _theta_map.size() < scan_ptr->ranges.size())
    {
      _theta_map.clear();

      for (int i=0; i<scan_ptr->ranges.size(); ++i)
        _theta_map.push_back( angle_min + double(i) * angle_increment );
    }

    for (int i=0; i<scan_ptr->ranges.size(); ++i)
    {
      double range = (double)scan_ptr->ranges[i];

      if (!_use_max_range && range >= max_range)  continue;
      if (std::isnan(range) || std::isinf(range)) continue;

      Point2D p = Point2D::makePoint2DFromPolarCoordinates(range, _theta_map[i]);

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
      ROS_DEBUG("Init. KF scan.");
      _kf_scan  = _scan;
      _new_scan = false;
      return;
    }

    _eval_scan = _scan;

    // Compute tranform that maps kf_scan onto scan.
    _sim_rel = _matcher.computeTransform(_kf_scan, _eval_scan, _correspondences).inv();

    if(std::abs(1. - _sim_rel.getScale()) > 0.1)
    {
      ROS_RED_STREAM("Something's Fishy.");
      //_new_scan = false;

      //return;
    }

    publish();

    if (_hasMovedEnough(_sim_rel))
    {
      ROS_DEBUG_STREAM("Update KF scan.");
      _updateKFScan();
    }

    ROS_DEBUG_STREAM("Estimated sim : \n\t x: " << _sim_rel.getX() <<
                     " y: " << _sim_rel.getY() << " theta: " << _sim_rel.getTheta()
                     << " scale: " << _sim_rel.getScale());

    _new_scan = false;
  }

  void publish()
  {
    tf::Transform rel_lf = _simToTf(_sim_rel);

    rel_lf = _base_to_laser * rel_lf * _laser_to_base;

    Similitude sim_pub = _sim_cumu * _sim_rel;

    // get pose (relative to last kf) in laser frame
    tf::Transform pose_lf = _simToTf(sim_pub);

    // get pose (relative to last kf) in base frame
    tf::Transform pose_bf = _laser_to_base * pose_lf;

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

    // What's the point if nobody cares
    if (_pub_scan.getNumSubscribers() > 0)
    {
      // Publish KF scan (green)
      visualization_msgs::Marker marker_kf, marker_scan, marker_kf_tr, corres;
      marker_kf.header.frame_id = "base_laser_link";
      marker_kf.header.stamp = ros::Time::now();
      marker_kf.id = 0;
      marker_kf.ns = "kf_scan";
      marker_kf.type = visualization_msgs::Marker::SPHERE_LIST;
      marker_kf.action = visualization_msgs::Marker::ADD;
      marker_kf.pose.position.x = 0.0;
      marker_kf.pose.position.y = 0.0;
      marker_kf.pose.position.z = 0.0;
      marker_kf.scale.x = 0.05;
      marker_kf.scale.y = 0.05;
      marker_kf.scale.z = 0.05;
      marker_kf.color.r = 0;
      marker_kf.color.g = 1.0;
      marker_kf.color.b = 0;
      marker_kf.color.a = 1.0;
      marker_kf.lifetime = ros::Duration(0);

      marker_scan.header   = marker_kf_tr.header   = corres.header   = marker_kf.header;
      marker_scan.id       = marker_kf_tr.id       = corres.id       = marker_kf.id;
      marker_scan.action   = marker_kf_tr.action   = corres.action   = marker_kf.action;
      marker_scan.pose     = marker_kf_tr.pose     = corres.pose     = marker_kf.pose;
      marker_scan.scale    = marker_kf_tr.scale    = corres.scale    = marker_kf.scale;
      marker_scan.lifetime = marker_kf_tr.lifetime = corres.lifetime = marker_kf.lifetime;

      for (size_t i=0; i<_kf_scan.size(); ++i)
      {
        geometry_msgs::Point point;

        point.x = _kf_scan[i].getX();
        point.y = _kf_scan[i].getY();
        marker_kf.points.push_back(point);
      }

      _pub_scan.publish(marker_kf);

      marker_scan, marker_kf_tr, corres;

      // Publish current scan (red)
      marker_scan.ns = "scan";
      marker_scan.type = visualization_msgs::Marker::SPHERE_LIST;
      marker_scan.color.r = 1.0;
      marker_scan.color.g = 0;
      marker_scan.color.b = 0;
      marker_scan.color.a = 1.0;

      for (size_t i=0; i<_eval_scan.size(); ++i)
      {
        geometry_msgs::Point point;

        point.x = _eval_scan[i].getX();
        point.y = _eval_scan[i].getY();
        point.z = 1.;
        marker_scan.points.push_back(point);
      }

      _pub_scan.publish(marker_scan);

      // Publish current scan corrected (blue)
      Scan scan_tr = _matcher.transformScan(_eval_scan, _sim_rel);

      marker_kf_tr.ns = "kf_scan_tr";
      marker_kf_tr.type = visualization_msgs::Marker::SPHERE_LIST;
      marker_kf_tr.color.r = 0;
      marker_kf_tr.color.g = 0;
      marker_kf_tr.color.b = 1.0;
      marker_kf_tr.color.a = 1.0;

      for (size_t i=0; i<scan_tr.size(); ++i)
      {
        geometry_msgs::Point point;

        point.x = scan_tr[i].getX();
        point.y = scan_tr[i].getY();
        point.z = 2.;
        marker_kf_tr.points.push_back(point);
      }

      _pub_scan.publish(marker_kf_tr);

      corres.ns = "correspondences";
      corres.type = visualization_msgs::Marker::LINE_LIST;
      corres.scale.x = 0.01; corres.scale.y = 0.01; corres.scale.z = 0.01;
      corres.color.r = 1.;
      corres.color.g = 0.5;
      corres.color.b = 0;
      corres.color.a = 1.0;

      for (size_t i=0; i<_correspondences.size(); ++i)
      {
        geometry_msgs::Point point_scan, point_kf_scan;

        point_kf_scan.x = _kf_scan[_correspondences[i].first].getX();
        point_kf_scan.y = _kf_scan[_correspondences[i].first].getY();
        point_kf_scan.z = 0.;

        point_scan.x    = _eval_scan[_correspondences[i].second].getX();
        point_scan.y    = _eval_scan[_correspondences[i].second].getY();
        point_scan.z    = 1.;

        corres.points.push_back(point_kf_scan);
        corres.points.push_back(point_scan);
      }

      _pub_scan.publish(corres);
    }
  }

private:

  bool _use_max_range, _new_scan, _pose_stamped;

  int _throttle;

  double _kf_min_dist, _kf_min_ang;

  std::string _base_frame, _laser_frame, _pose_frame;

  std::vector<double> _theta_map;

  ros::Time _stamp;
  Scan _kf_scan, _scan, _eval_scan;

  Similitude _sim_rel, _sim_cumu;
  Correspondences _correspondences;

  ros::NodeHandle _nh;

  ros::Subscriber _sub_scan;
  ros::Publisher  _pub_pose;
  ros::Publisher  _pub_pose_stamped;

  ros::Publisher  _pub_scan;

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

    ROS_DEBUG_STREAM("Param '_use_max_range' set to : " << _use_max_range);
    ROS_DEBUG_STREAM("Param '_kf_min_dist' set to   : " << _kf_min_dist);
    ROS_DEBUG_STREAM("Param '_kf_min_ang' set to    : " << _kf_min_ang);
    ROS_DEBUG_STREAM("Param '_throttle' set to      : " << _throttle);
  }

  bool _hasMovedEnough(const Similitude& sim)
  {
    if (std::sqrt(sim.getX()*sim.getX() + sim.getY()*sim.getY()) > _kf_min_dist) return true;

    if (std::fabs(sim.getTheta()) > _kf_min_ang) return true;

    return false;
  }

  void _updateKFScan()
  {
    _kf_scan = _scan;

    tf::Transform rel_lf = _simToTf(_sim_rel);

    rel_lf = _base_to_laser * rel_lf * _laser_to_base;

    _sim_cumu *= _tfToSim(rel_lf);
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

  bool _getInitialPoseTf(Similitude& sim)
  {
    tf::StampedTransform map_to_base;
    map_to_base.setIdentity();

    bool got_tf = false;
    try
    {
      _listener.waitForTransform(
            _pose_frame, _base_frame, ros::Time(0), ros::Duration(2.0));
      _listener.lookupTransform (
            _pose_frame, _base_frame, ros::Time(0), map_to_base);

      got_tf = true;
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("Could not get initial transform from base to laser frame, %s", ex.what());
    }

    ROS_DEBUG_STREAM("Init tf : \n\t x: " << map_to_base.getOrigin().getX() <<
                     " y: " << map_to_base.getOrigin().getY()
                     << " theta: " << tf::getYaw( map_to_base.getRotation() ));

    sim.setX(map_to_base.getOrigin().getX());
    sim.setY(map_to_base.getOrigin().getY());

    sim.setTheta(tf::getYaw( map_to_base.getRotation() ));

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

  tf::Transform _simToTf(const Similitude& sim)
  {
    tf::Transform t;
    t.setOrigin(tf::Vector3(sim.getX(), sim.getY(), 0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, sim.getTheta());
    t.setRotation(q);

    return t;
  }

  Similitude _tfToSim(const tf::Transform& tfin)
  {
    return Similitude(tfin.getOrigin().getX(),
                      tfin.getOrigin().getY(),
                      tf::getYaw(tfin.getRotation()), 1.);
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

  ros::Rate rate(25);

  while (ros::ok())
  {
    ros::Time start = ros::Time::now();

    scan_matcher.process();

    ROS_DEBUG_STREAM("Took : " << (ros::Time::now() - start).toSec() << " to process.\n");

    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}
