#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <vector>

#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Core"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


//struct trajectory {
//  VectorXd s_coef;
//  VectorXd d_coef;
//  double T;
//};


// Evaluate a polynomial.
inline double polyeval(VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
inline VectorXd polyfit(VectorXd xvals, VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


// Differentiate a polynomial.
inline VectorXd polyder(VectorXd coeffs) {
  int order = coeffs.size();
  VectorXd der(max(order - 1, 1));
  der[0] = 0;
  for (int i = 0; i < order - 1; i++) {
    der[i] = coeffs[i + 1] * (i + 1);
  }
  return der;
}


// Adapted from L5.30 - ptg.py
inline double logistic(double x) {
  /*
  A function that returns a value between 0 and 1 for x in the
  range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

  Useful for cost functions.
  */
  return 2 / ( 1 + exp(-x) ) - 1;
}


inline double distance_2d(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}



// Functions below were moved from main.cpp in project starter code.

// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}
inline double deg2rad(double x) {
  return x * pi() / 180;
}
inline double rad2deg(double x) {
  return x * 180 / pi();
}


inline double safe_atan2(double y, double x) {
  double epsilon = 1e-4;
  double absx = abs(x);
  double absy = abs(y);

  double result;
  if ((absx >= epsilon) & (absy >= epsilon)) {
    result = atan2(y, x);
  } else if ((absx < epsilon) & (absy >= epsilon)) {
    result = (y / absy) *  pi()/2;
  } else if ((absx >= epsilon) & (absy < epsilon)) {
    result = 0;
  } else {
    result = 0;
  }
  return result;
}

#endif
