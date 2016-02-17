#include <boost/foreach.hpp>

#include <eigen3/Eigen/Dense>

#include "simple_scan_matcher/similitude.h"

namespace simple_scan_matcher
{

typedef std::pair<uint, uint>   IndexPair;
typedef std::vector<IndexPair>  IndexPairs;

  void Similitude::computeSimilitude(Complex a, Complex b, Complex ap, Complex bp,
                                     ComplexVector2& simvec)
  {
    Complex aden = (!(a == b))? (a - b) : Complex(1.,1.);

    Complex A = (ap - bp) / aden;
    Complex B = ap - (A*a);

    simvec(0) = A;
    simvec(1) = B;

    return;
  }

  void Similitude::computeSimilitude(Complex a, Complex b, Complex ap, Complex bp,
                                     Similitude& sim)
  {
    ComplexVector2 vecsim;
    computeSimilitude(a, b, ap, bp, vecsim);

    sim = Similitude(vecsim(0), vecsim(1));

    return;
  }

  double Similitude::computeSimilitude(const Scan& source, const Scan& target,
                                       const Correspondences& correspondences,
                                       ComplexVector2& simvec)
  {
    simvec.setZero();
    simvec(0) = Complex(0,1);

    if (source.empty() || target.empty() || correspondences.empty())
      return 1e17;

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

    ComplexMatrix A; A.resize(corr_size, 2);
    ComplexVector B; B.resize(corr_size);

    for (uint i=0; i<corr_size; ++i)
    {
      A(i,0) = C_ONE;
      A(i,1) = Complex(source[correspondences[i].first].getX(),
                       source[correspondences[i].first].getY());

      B(i)   = Complex(target[correspondences[i].second].getX(),
                       target[correspondences[i].second].getY());
    }

    simvec = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

    return (A*simvec - B).norm() / B.norm();
  }

  double Similitude::computeSimilitude(const Scan& source, const Scan& target,
                                       const Correspondences& correspondences,
                                       Similitude& sim)
  {
    ComplexVector2 simvec;
    double lsqe = computeSimilitude(source, target, correspondences, simvec);
    sim = Similitude(simvec(0), simvec(1));
    return lsqe;
  }

  double Similitude::computeSimilitude(const Scan& source, const Scan& target,
                                       const Correspondences& correspondences,
                                       const ComplexDiagonalMatrix& weights,
                                       ComplexVector2& simvec)
  {
    simvec.setZero();
    simvec(0) = Complex(0,1);

    if (source.empty() || target.empty() || correspondences.empty())
      return 1e17;

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

    simvec = (weights*A).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(weights*B);

    return (A*simvec - B).norm() / B.norm();
  }

  double Similitude::computeSimilitude(const Scan& source, const Scan& target,
                                       const Correspondences& correspondences,
                                       const ComplexDiagonalMatrix& weights,
                                       Similitude& sim)
  {
    ComplexVector2 simvec;
    double lsqe = computeSimilitude(source, target, correspondences, weights, simvec);
    sim = Similitude(simvec(0), simvec(1));
    return lsqe;
  }

} //namespace simple_scan_matcher
