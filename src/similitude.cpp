#include <boost/foreach.hpp>

#include "simple_scan_matcher/similitude.h"

namespace simple_scan_matcher
{

  Similitude Similitude::computeSimilitude(const Scan& source, const Scan& target,
                                           const Correspondences& correspondences)
  {
    if (source.empty() || target.empty() || correspondences.empty())
      return Similitude();

    Similitude::ScanComplex c_source, c_target;

    for (int i=0; i<source.size(); ++i)
      c_source.push_back( Similitude::Complex(source[i].getX(), source[i].getY()) );

    for (int i=0; i<target.size(); ++i)
      c_target.push_back( Similitude::Complex(target[i].getX(), target[i].getY()) );

    ceres::Problem problem;

    double a[2] = {0, 0};
    double b[2] = {0, 0};

    for (size_t i=0; i<correspondences.size(); ++i)
    {
      ceres::CostFunction* cost_function =
             new ceres::AutoDiffCostFunction<SimSolver::ComplexCostFunctor, 2, 2, 2>(
                 new SimSolver::ComplexCostFunctor( c_source[correspondences[i].first],
                                                    c_target[correspondences[i].second] ) );

      problem.AddResidualBlock(cost_function, NULL, &a[0], &b[0] );
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.gradient_tolerance = 1e-15;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Complex ac(a[0], a[1]);
    Complex bc(b[0], b[1]);

    return Similitude(ac, bc);
  }

} //namespace simple_scan_matcher

//  transformation.x     = b.real();
//  transformation.y     = b.imag();
//  transformation.theta = std::atan2(a.imag(), a.real());
//  transformation.scale = std::sqrt(a.real()*a.real() + a.imag()*a.imag());
