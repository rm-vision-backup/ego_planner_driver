#include "ego_planner_driver/polynomial_traj.h"

#include <cmath>

namespace ego_planner_driver
{

Eigen::Vector3d OneSegmentPolynomial::evaluate(double t) const
{
  Eigen::Vector3d p = coeff[5];
  for (int i = 4; i >= 0; --i)
    p = p * t + coeff[i];  // Horner, ascending powers
  return p;
}

Eigen::Vector3d OneSegmentPolynomial::evaluateVel(double t) const
{
  Eigen::Vector3d v = coeff[5] * 5.0;
  for (int i = 4; i >= 1; --i)
    v = v * t + coeff[i] * double(i);
  return v;
}

Eigen::Vector3d OneSegmentPolynomial::evaluateAcc(double t) const
{
  Eigen::Vector3d a = coeff[5] * 20.0;
  for (int i = 4; i >= 2; --i)
    a = a * t + coeff[i] * double(i * (i - 1));
  return a;
}

OneSegmentPolynomial oneSegmentTrajGen(const Eigen::Vector3d& start_pt,
                                       const Eigen::Vector3d& start_vel,
                                       const Eigen::Vector3d& start_acc,
                                       const Eigen::Vector3d& end_pt,
                                       const Eigen::Vector3d& end_vel,
                                       const Eigen::Vector3d& end_acc,
                                       double t)
{
  // Mapping C (6x6): rows = [p(0), v(0), a(0), p(t), v(t), a(t)] as linear
  // functions of the ascending-power coefficients c0..c5.
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6);
  C(0, 0) = 1.0;  // p(0) = c0
  C(1, 1) = 1.0;  // v(0) = c1
  C(2, 2) = 2.0;  // a(0) = 2*c2
  C(3, 0) = 1.0;  // p(t) = c0 + c1*t + ... + c5*t^5
  C(3, 1) = t;
  C(3, 2) = t * t;
  C(3, 3) = t * t * t;
  C(3, 4) = t * t * t * t;
  C(3, 5) = t * t * t * t * t;
  C(4, 1) = 1.0;  // v(t) = c1 + 2*c2*t + 3*c3*t^2 + 4*c4*t^3 + 5*c5*t^4
  C(4, 2) = 2.0 * t;
  C(4, 3) = 3.0 * t * t;
  C(4, 4) = 4.0 * t * t * t;
  C(4, 5) = 5.0 * t * t * t * t;
  C(5, 2) = 2.0;  // a(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + 20*c5*t^3
  C(5, 3) = 6.0 * t;
  C(5, 4) = 12.0 * t * t;
  C(5, 5) = 20.0 * t * t * t;

  Eigen::VectorXd bx(6), by(6), bz(6);
  bx << start_pt(0), start_vel(0), start_acc(0), end_pt(0), end_vel(0), end_acc(0);
  by << start_pt(1), start_vel(1), start_acc(1), end_pt(1), end_vel(1), end_acc(1);
  bz << start_pt(2), start_vel(2), start_acc(2), end_pt(2), end_vel(2), end_acc(2);

  const Eigen::VectorXd cx = C.colPivHouseholderQr().solve(bx);
  const Eigen::VectorXd cy = C.colPivHouseholderQr().solve(by);
  const Eigen::VectorXd cz = C.colPivHouseholderQr().solve(bz);

  OneSegmentPolynomial poly;
  for (int i = 0; i < 6; ++i)
    poly.coeff[i] = Eigen::Vector3d(cx(i), cy(i), cz(i));
  return poly;
}

}  // namespace ego_planner_driver
