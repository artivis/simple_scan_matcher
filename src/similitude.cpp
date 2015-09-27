#include <boost/foreach.hpp>

#include "simple_scan_matcher/similitude.h"

namespace simple_scan_matcher
{

  Similitude Similitude::computeSimilitude(const Scan& source, const Scan& target)
  {
    if (source.empty() || target.empty()) return Similitude();

    /*
    Similitude::SimMatVec c_source(source.size()), c_target(target.size());

    for (int i=0; i<source.size(); ++i)
      c_source(i) = Similitude::Complex(source[i].getX(), source[i].getY());

    for (int i=0; i<target.size(); ++i)
      c_target(i) = Similitude::Complex(target[i].getX(), target[i].getY());
    */

    Similitude::ScanComplex c_source(source.size()), c_target(target.size());

    for (int i=0; i<source.size(); ++i)
      c_source.push_back( Similitude::Complex(source[i].getX(), source[i].getY()) );

    for (int i=0; i<target.size(); ++i)
      c_target.push_back( Similitude::Complex(target[i].getX(), target[i].getY()) );

    // TODO: find correspondences a.k.a closest neigbor ?

    ceres::Problem problem;

    Similitude::Complex a; a.imag() = 0.; a.real() = 0.;
    Similitude::Complex b; b.imag() = 0.; b.real() = 0.;

//    for (int i=0; i<correspondences.size(); ++i)
//    {
//      ceres::CostFunction* cost_function =
//             new ceres::AutoDiffCostFunction<SimSolver::ComplexCostFunctor, 1, 1, 1>(
//                 new SimSolver::ComplexCostFunctor( corres[i].first, corres[i].second ));

//      problem.AddResidualBlock(cost_function, NULL, &a, &b);
//    }

  }

} //namespace simple_scan_matcher

//void toto()
//{
//  int corr_size = correspondences.size();

//  // go to complex plan so that
//  // p(2,6) -> pc = 2 + 6i
//  complex_vec point_set_a, point_set_b;

//  complex_pair_vec corr_comp;
//  corr_comp.reserve(corr_size);

//  point_set_a.reserve(corr_size);
//  point_set_b.reserve(corr_size);

//  for (uint i=0; i<corr_size; ++i)
//  {
//    complex point_a(correspondences[i].first.x,  correspondences[i].first.y);
//    complex point_b(correspondences[i].second.x, correspondences[i].second.y);

//    point_set_a.push_back( point_a );
//    point_set_b.push_back( point_b );

//    corr_comp.push_back( std::make_pair(point_a, point_b) );
//  }

//  // Similarity (geometry) computed from
//  // 2 pairs of corresponding points
//  // compute all possible combinations of those pairs
//  index_pair_vec comb_index;
//  for (uint i=0; i<corr_size; ++i)
//    for (uint j=i+1; j<corr_size; ++j)
//      comb_index.push_back( std::make_pair(i, j) );

//  // compute the actual Similarity
//  complex a, b;
//  for (uint i=0; i<comb_index.size(); ++i)
//  {
//    complex& az  = corr_comp[comb_index[i].first].first;
//    complex& azp = corr_comp[comb_index[i].first].second;

//    complex& bz  = corr_comp[comb_index[i].second].first;
//    complex& bzp = corr_comp[comb_index[i].second].second;

//    a += (azp - bzp) / (az - bz);

//    complex a_tmp(a);
//    double& aimg  = a_tmp.imag();
//    double& areal = a_tmp.real();
//    aimg  /= double(i+1);
//    areal /= double(i+1);

//    b += azp - (a_tmp*az);
//  }

//  double den = double(comb_index.size());

//  double& aimg  = a.imag();
//  double& areal = a.real();
//  double& bimg  = b.imag();
//  double& breal = b.real();
//  aimg  /= den;
//  areal /= den;
//  bimg  /= den;
//  breal /= den;

//  transformation.x     = b.real();
//  transformation.y     = b.imag();
//  transformation.theta = std::atan2(a.imag(), a.real());
//  transformation.scale = std::sqrt(a.real()*a.real() + a.imag()*a.imag());

//  //  std::cout << "compute2DPose2 : " << transformation << std::endl;
//}
