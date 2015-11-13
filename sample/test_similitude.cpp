#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>

#include <boost/foreach.hpp>

#include <math.h>

#include "simple_scan_matcher/scan_matcher.h"

using namespace simple_scan_matcher;

double errorRotation(const Similitude::SimMat& a,
                     const Similitude::SimMat& b)
{
  Eigen::Matrix2d a_r, b_r;

  a_r << a(0,0),a(0,1),a(1,0),a(1,1);
  b_r << b(0,0),b(0,1),b(1,0),b(1,1);

  Eigen::Matrix2d I; I << 1, 0, 0, 1;
  Eigen::Matrix2d I_est = a_r * b_r.transpose();

  return (I_est - I).norm();
}

double errorTranslation(const Similitude::SimMat& a,
                        const Similitude::SimMat& b)
{
  Eigen::Vector2d a_t, b_t;

  a_t << a(0,2),a(1,2);
  b_t << b(0,2),b(1,2);

  return (a_t - b_t).norm();
}

int main(int argc, char **argv)
{
  bool add_noise = true;

  Scan scan_orig, scan_trans;

  for (size_t i=1; i<=20; ++i)
    scan_orig.push_back( Point2D(i, 2.*i ) );

  Similitude gt(0.5, 0.3333, 0.11, 1.043);

  ScanMatcher scan_matcher;

  scan_trans = scan_matcher.transformScan(scan_orig, gt);

  // add noise
  if (add_noise)
  {
    for (size_t i=0; i<scan_trans.size(); ++i)
    {
      Eigen::Vector3d noise;

      for (int r=0; r<2; ++r)
      {
        double R1 = (double) rand() / (double) RAND_MAX;
        double R2 = (double) rand() / (double) RAND_MAX;
        double X  = (double) sqrt( -2. * log( R1 )) * cos( 2. * M_PI * R2 );

        noise(r) = X / 100.;
      }

      noise(3) = 0;

      scan_trans[i] += noise;
    }
  }

  Similitude est = scan_matcher.computeTransform(scan_orig, scan_trans);

  std::cout << "Source scan :\n" << scan_orig  << std::endl;
  std::cout << "\nTarget scan :\n" << scan_trans << std::endl;

  std::cout << "\nGround truth Similarity :\n\t" << gt  << std::endl;
  std::cout << "Estimated Similarity      :\n\t" << est << std::endl;

  std::cout << "\nGround truth T :\n" << gt.getTransformationMat()  << std::endl;
  std::cout << "Estimated T      :\n" << est.getTransformationMat() << std::endl;

  std::cout << "\nRotation    error : " << errorRotation(gt.getTransformationMat(),    est.getTransformationMat()) << std::endl;
  std::cout << "Translation error : "   << errorTranslation(gt.getTransformationMat(), est.getTransformationMat()) << std::endl;

  return 0;
}
