#ifndef SIMPLE_SCAN_MATCHER_TRANSFORMATION2D_H
#define SIMPLE_SCAN_MATCHER_TRANSFORMATION2D_H

#include <complex>

#include <eigen3/Eigen/Core>
//#include <eigen/Eigen/StdVector>

#include <ceres/ceres.h>

#include "simple_scan_matcher/point2d.h"

namespace simple_scan_matcher
{

  class Similitude
  {
    typedef Eigen::Matrix2cd                 SimMat;
    typedef Eigen::aligned_allocator<SimMat> SimMatAlloc;
    typedef std::vector<SimMat, SimMatAlloc> SimMatVec;

  public:

    typedef std::complex<double> Complex;
    typedef std::vector<Complex> ScanComplex;

    Similitude() : _x(0), _y(0), _theta(0), _scale(0) { }

    Similitude(double x, double y, double theta, double scale) :
      _x(x), _y(y), _theta(theta), _scale(scale) { }

    ~Similitude() { }

    Similitude computeSimilitude(const Scan& source, const Scan& target);

  //private:

    double _x, _y, _theta, _scale;
  };

  namespace SimSolver
  {
//    using ceres::AutoDiffCostFunction;
//    using ceres::CostFunction;
//    using ceres::Problem;
//    using ceres::Solver;

    struct ComplexCostFunctor
    {
      ComplexCostFunctor(Similitude::Complex z, Similitude::Complex zp)
        : _z(z), _zp(zp) {}

       bool operator()(const Similitude::Complex* const a, const Similitude::Complex* const b, double* residual) const
       {
         Similitude::Complex t = _z - ((*a)*_zp+(*b));

         residual[0] = (t * std::conj(t)).real();
         return true;
       }

    private:

       Similitude::Complex _z, _zp;
    };

  }

} //namespace simple_scan_matcher

#endif //SIMPLE_SCAN_MATCHER_TRANSFORMATION2D_H
