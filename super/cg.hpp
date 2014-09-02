#ifndef SUPER__CG_HPP
#define SUPER__CG_HPP

#include <Eigen/Dense>
#include <limits>

template<typename Atype, typename Scalar>
class GeneralCGSolver {
public:
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> Vector;

  GeneralCGSolver(const Atype &A, const Vector &b, Vector &x)
  : mA(A), mb(b), mx(x), mIt(0) {
    mR = b - A*x;
    mRNorm2 = mR.squaredNorm();
    mP = mR;
  } 

  unsigned int iterations() const { return mIt; }
  const Scalar &ssq_residual() const { return mRNorm2; }

  void step() {
    Vector Ap(mA*mP);
    Scalar alpha = mRNorm2 / mP.dot(Ap);
    mx += alpha * mP;
    mR -= alpha * Ap;
    Scalar newRNorm2 = mR.squaredNorm();
    Scalar beta = newRNorm2 / mRNorm2;
    mRNorm2 = newRNorm2;
    mP *= beta;
    mP += mR;
    mIt++;
  }

  void solve(const Scalar tol = std::numeric_limits<Scalar>::epsilon()) {
    while(1) {
      this->step();
      if (mRNorm2 < tol || mIt >= mx.size()) {
        break;
      }
    }
  }

private:
  const Atype  &mA;
  const Vector &mb;
  Vector       &mx;

  Vector       mR, mP;
  Scalar       mRNorm2;
  unsigned int mIt;
};

/**
 * Implement multiplication by the matrix which results from the normal
 * equations of a regularized least squares problem.  I.E., this computes
 * (A^T * A + Q) * x  without needing to explicitly compute the matrix 
 * (A^T * A + Q)
 **/
template<typename Atype, typename Qtype>
class RegLSQMatrix {
public:
  RegLSQMatrix(const Atype &A, const Qtype &Q)
  : mA(A), mQ(Q) {}

  template<typename Matrix>
  inline Matrix operator*(const Matrix &x) const {
    return (mA.transpose()*(mA*x)) + (mQ*x);
  }

private:
  const Atype &mA;
  const Qtype &mQ;
};


#endif

