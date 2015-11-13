#include <boost/foreach.hpp>

#include <eigen3/Eigen/Dense>

#include "simple_scan_matcher/similitude.h"

namespace simple_scan_matcher
{

typedef std::pair<uint, uint>   IndexPair;
typedef std::vector<IndexPair>  IndexPairs;

  void Similitude::computeSimilitude(Complex a, Complex b, Complex ap, Complex bp,
                                     ComplexVec& vec)
  {
    Similitude::Complex aden = (!(a == b))? (a - b) : Similitude::Complex(1.,1.);

    Complex A = (ap - bp) / aden;
    Complex B = ap - (A*a);

    vec.push_back(A);
    vec.push_back(B);

    return;
  }

  double Similitude::computeSimilitude(const Scan& source, const Scan& target,
                                       const Correspondences& correspondences,
                                       ComplexVec& vec)
  {
    vec.clear();

    if (source.empty() || target.empty() || correspondences.empty())
    {
      vec.push_back(Complex(0,1));
      vec.push_back(Complex(0,0));
      return 1e17;
    }

    Similitude::ScanComplex c_source, c_target;

    int corr_size = correspondences.size();

    for (int i=0; i<corr_size; ++i)
    {
      c_source.push_back( Similitude::Complex(source[correspondences[i].first].getX(),
                                              source[correspondences[i].first].getY()) );

      c_target.push_back( Similitude::Complex(target[correspondences[i].second].getX(),
                                              target[correspondences[i].second].getY()) );
    }

    assert(c_source.size() == c_target.size());

    Eigen::MatrixXcd A; A.resize(corr_size, 2);
    Eigen::VectorXcd B; B.resize(corr_size);

    for (uint i=0; i<corr_size; ++i)
    {
      A(i,0) = C_ONE;
      A(i,1) = Complex(source[correspondences[i].first].getX(),
                       source[correspondences[i].first].getY());

      B(i)   = Complex(target[correspondences[i].second].getX(),
                       target[correspondences[i].second].getY());
    }

    Eigen::Vector2cd T = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

    vec.push_back(T(1));
    vec.push_back(T(0));

    return (A*T - B).norm() / B.norm();

//    ceres::Problem problem;

//    // TODO : init
//    // a can't be null.
//    // this gives a first guess as
//    // scale = 1, theta = 0
//    double a[2] = {0., 1.};
//    double b[2] = {0., 0.};

//    for (size_t i=0; i<c_source.size(); ++i)
//    {
//      ceres::CostFunction* cost_function =
//             new ceres::AutoDiffCostFunction<SimSolver::ComplexCostFunctor, 2, 2, 2>(
//                 new SimSolver::ComplexCostFunctor( c_source[i], c_target[i] ) );

//      problem.AddResidualBlock(cost_function, /*new ceres::CauchyLoss(0.5)*/ NULL, &a[0], &b[0] );
//    }

//    ceres::Solver::Options options;
//    options.linear_solver_type = ceres::DENSE_QR;
//    options.minimizer_progress_to_stdout = false;
//    options.gradient_tolerance = 1e-15;

//    options.logging_type = ceres::SILENT;

//    ceres::Solver::Summary summary;
//    ceres::Solve(options, &problem, &summary);

//    if (a[0] == 0 && a[1] == 0) a[1] = 1.;

//    Complex ac(a[0], a[1]);
//    Complex bc(b[0], b[1]);

//    vec.push_back(ac);
//    vec.push_back(bc);

//    return;
  }

  double Similitude::computeSimilitude(const Scan& source, const Scan& target,
                                       const Correspondences& correspondences,
                                       Similitude& sim)
  {
    ComplexVec vec;
    double lsqe = computeSimilitude(source, target, correspondences, vec);
    sim = Similitude(vec[0], vec[1]);
    return lsqe;
  }

} //namespace simple_scan_matcher
