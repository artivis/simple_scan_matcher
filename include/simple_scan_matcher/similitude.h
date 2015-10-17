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
  public:

    typedef Eigen::Matrix3d SimMat;

    typedef std::complex<double> Complex;
    typedef std::vector<Complex> ScanComplex;

    Similitude() : _x(0), _y(0), _theta(0), _scale(0) { }

    Similitude(Complex a, Complex b) :
      _x(b.real()), _y(b.imag()),
      _theta(std::atan2(a.imag(), a.real())),
      _scale(std::sqrt(a.real()*a.real() + a.imag()*a.imag())) { }

    Similitude(double x, double y, double theta, double scale) :
      _x(x), _y(y), _theta(theta), _scale(scale) { }

    Similitude(Similitude& s) :
      _x(s.getX()), _y(s.getY()), _theta(s.getTheta()), _scale(s.getScale()) { }

    Similitude(const Similitude& s) :
      _x(s.getX()), _y(s.getY()), _theta(s.getTheta()), _scale(s.getScale()) { }

    ~Similitude() { }

    static Similitude computeSimilitude(const Scan& source, const Scan& target,
                                        const Correspondences& correspondences);

    inline SimMat getSimilarityMat() const
    {
      SimMat sim_mat; //homogenous transform
      sim_mat << std::cos(_theta), -std::sin(_theta), _scale * _x,
                 std::sin(_theta),  std::cos(_theta), _scale * _y,
                 0, 0, 1;

      return sim_mat;
    }

    Similitude& operator=(const Similitude& lhs)
    {
      _x = lhs.getX();
      _y = lhs.getY();
      _theta = lhs.getTheta();
      _scale = lhs.getScale();

      return *this;
    }

//    Similitude& operator=(const SimMat& lhs)
//    {
//      _x = lhs(0,2);
//      _y = lhs(1,2);
//      _theta = std::atan2(lhs(1,0), lhs(0,0));
//      _scale = 1.;

//      return *this;
//    }

//    Similitude& operator*(const Similitude& lhs)
//    {
////      _a = lhs._a;
////      _b = lhs._b;
//      _x = lhs.getX();
//      _y = lhs.getY();
//      _theta = lhs.getTheta();
//      _scale = lhs.getScale();

//      return *this;
//    }

//    friend Similitude operator*(const Similitude& rhs, const Similitude& lhs)
//    {
//      SimMat sm = rhs.getSimilarityMat() * lhs.getSimilarityMat();
//      Similitude s(lhs(0,2), lhs(1,2), std::atan2(lhs(1,0), lhs(0,0)));
//      return s;
//    }

    inline double getX() const { return _x; }
    inline void setX(double x) { _x = x; }

    inline double getY() const { return _y; }
    inline void setY(double y) { _y = y; }

    inline double getTheta() const { return _theta; }
    inline void setTheta(double theta) { _theta = theta; }

    inline double getScale() const { return _scale; }
    inline void setScale(double scale) { _scale = scale; }

  private:

    double _x, _y, _theta, _scale;
  };

  namespace SimSolver
  {
    struct ComplexCostFunctor
    {
      ComplexCostFunctor(Similitude::Complex z, Similitude::Complex zp)
        : _z(z), _zp(zp) { }

      template <typename T>
       bool operator()(const T* const a, const T* const b, T* residual) const
       {
         const std::complex<T> a_c(a[0], a[1]);
         const std::complex<T> b_c(b[0], b[1]);

         const std::complex<T> z(_z);
         const std::complex<T> zp(_zp);

         const std::complex<T> t = zp - (a_c*z+b_c);

         residual[0] = T(t.real());
         residual[1] = T(t.imag());

         return true;
       }

    private:

       Similitude::Complex _z, _zp;
    };

  }

} //namespace simple_scan_matcher

inline std::ostream& operator<<(std::ostream& out, const simple_scan_matcher::Similitude& s)
{
   return out << "t_x " << s.getX() << " t_y " << s.getY() << " theta " << s.getTheta()
              << " scale " << s.getScale();
}

#endif //SIMPLE_SCAN_MATCHER_TRANSFORMATION2D_H
