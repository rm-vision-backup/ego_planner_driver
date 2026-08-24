#ifndef EGO_PLANNER_DRIVER_POLYNOMIAL_TRAJ_H_
#define EGO_PLANNER_DRIVER_POLYNOMIAL_TRAJ_H_

#include <Eigen/Eigen>
#include <vector>

namespace ego_planner_driver
{
// Minimal port of the upstream traj_utils::PolynomialTraj::one_segment_traj_gen:
// a single quintic polynomial connecting start/end pose, velocity and
// acceleration states — the minimum-snap solution for one segment.  The
// multi-segment minSnapTraj is intentionally NOT ported (backup plan only).
//
// Coefficients are stored in ascending power order: value(t) = sum_k c[k]*t^k.
struct OneSegmentPolynomial
{
  Eigen::Vector3d coeff[6];  // coeff[k] is the 3D coefficient of t^k

  Eigen::Vector3d evaluate(double t) const;
  Eigen::Vector3d evaluateVel(double t) const;
  Eigen::Vector3d evaluateAcc(double t) const;
};

// Closed-form single-segment min-snap (quintic) trajectory.  `t` is the segment
// duration and must be strictly positive and finite.
OneSegmentPolynomial oneSegmentTrajGen(const Eigen::Vector3d& start_pt,
                                       const Eigen::Vector3d& start_vel,
                                       const Eigen::Vector3d& start_acc,
                                       const Eigen::Vector3d& end_pt,
                                       const Eigen::Vector3d& end_vel,
                                       const Eigen::Vector3d& end_acc,
                                       double t);

}  // namespace ego_planner_driver

#endif  // EGO_PLANNER_DRIVER_POLYNOMIAL_TRAJ_H_
