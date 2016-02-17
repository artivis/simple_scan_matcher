#ifndef SIMPLE_SCAN_MATCHER_TRANSFORMATION2D_H
#define SIMPLE_SCAN_MATCHER_TRANSFORMATION2D_H

#include <complex>

#include <eigen3/Eigen/Core>

#include <ceres/ceres.h>

#include "simple_scan_matcher/point2d.h"

namespace simple_scan_matcher
{

  class Similitude
  {
  public:

    typedef Eigen::Matrix3d SimMat;
    typedef Eigen::Matrix2d RotMat;
    typedef Eigen::Vector2d TraVec;

    typedef std::complex<double> Complex;
    typedef std::vector<Complex> ComplexVec;
    typedef std::vector<Complex> ScanComplex;

    typedef Eigen::Vector2cd ComplexVector2;
    typedef Eigen::VectorXcd ComplexVector;
    typedef Eigen::MatrixXcd ComplexMatrix;

    typedef Eigen::DiagonalMatrix<Complex, Eigen::Dynamic> ComplexDiagonalMatrix;

    Similitude() : _x(0), _y(0), _theta(0), _scale(1) { }

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

    static void computeSimilitude(Complex a, Complex b, Complex ap, Complex bp,
                                  ComplexVec& vec);

    static double computeSimilitude(const Scan& source, const Scan& target,
                                    const Correspondences& correspondences,
                                    ComplexVec& vec);

    static double computeSimilitude(const Scan& source, const Scan& target,
                                    const Correspondences& correspondences,
                                    Similitude& sim);

    inline Similitude inv() const
    {
      return Similitude(-_x, -_y, -_theta, 1./_scale);
    }

    inline SimMat getTransformationMat() const
    {
      SimMat tran_mat; //homogenous transform
      tran_mat << std::cos(_theta), -std::sin(_theta), _x,
                  std::sin(_theta),  std::cos(_theta), _y,
                  0, 0, 1;

      return tran_mat;
    }

    inline SimMat getSimilarityMat() const
    {
      SimMat sim_mat;
      sim_mat << std::cos(_theta), -std::sin(_theta), _scale * _x,
                 std::sin(_theta),  std::cos(_theta), _scale * _y,
                 0, 0, 1;

      return sim_mat;
    }

    inline RotMat getRotationMat() const
    {
      RotMat rot_mat;
      rot_mat << std::cos(_theta), -std::sin(_theta),
                 std::sin(_theta),  std::cos(_theta);

      return rot_mat;
    }

    inline TraVec getTranslationVec() const
    {
      TraVec tra_vec;
      tra_vec << _x, _y;

      return tra_vec;
    }

    Similitude& operator=(const Similitude& lhs)
    {
      _x     = lhs.getX();
      _y     = lhs.getY();
      _theta = lhs.getTheta();
      _scale = lhs.getScale();

      return *this;
    }

    Similitude& operator*=(const Similitude& lhs)
    {
      SimMat mult = getTransformationMat() * lhs.getTransformationMat();
      _x = mult(0,2);
      _y = mult(1,2);
      _theta = std::atan2(mult(1,0), mult(0,0));
      _scale *= lhs.getScale();

      return *this;
    }

    friend Similitude operator*(const Similitude& rhs, const Similitude& lhs)
    {
      SimMat sm = rhs.getTransformationMat() * lhs.getTransformationMat();
      return Similitude( (double)sm(0,2), (double)sm(1,2),
                          std::atan2(sm(1,0), sm(0,0)),
                          rhs.getScale() * lhs.getScale() );
    }

    friend Similitude operator+(const Similitude& rhs, const Similitude& lhs)
    {
      return Similitude( rhs.getX()+lhs.getX(), rhs.getY()+lhs.getY(),
                         rhs.getTheta()+lhs.getTheta(),
                         rhs.getScale()*lhs.getScale() );
    }

    inline double getX() const         { return _x; }
    inline void setX(double x)         { _x = x; }

    inline double getY() const         { return _y; }
    inline void setY(double y)         { _y = y; }

    inline double getTheta() const     { return _theta; }
    inline void setTheta(double theta) { _theta = theta; }

    inline double getScale() const     { return _scale; }
    inline void setScale(double scale) { _scale = scale; }

  private:

    double _x, _y, _theta, _scale;
  };

  const Similitude::Complex C_ONE = Similitude::Complex(1,0);

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

  } //namespace SimSolver

} //namespace simple_scan_matcher

inline std::ostream& operator<<(std::ostream& out, const simple_scan_matcher::Similitude& s)
{
   return out << "t_x " << s.getX() << " t_y " << s.getY() << " theta " << s.getTheta()
              << " scale " << s.getScale();
}

#endif //SIMPLE_SCAN_MATCHER_TRANSFORMATION2D_H
