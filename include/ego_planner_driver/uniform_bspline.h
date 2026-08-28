#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <cstddef>
#include <iostream>

using namespace std;

namespace ego_planner
{
  // An implementation of non-uniform B-spline with different dimensions
  // It also represents uniform B-spline which is a special case of non-uniform
  class UniformBspline
  {
  private:
    // control points for B-spline with different dimensions.
    // Each row represents one single control point
    // The dimension is determined by column number
    // e.g. B-spline with N points in 3D space -> Nx3 matrix
    Eigen::MatrixXd control_points_;

    int p_, n_, m_;     // p degree, n+1 control points, m = n+p+1
    Eigen::VectorXd u_; // knots vector
    double interval_;   // knot span \delta t

    Eigen::MatrixXd getDerivativeControlPoints();

    double limit_vel_, limit_acc_, limit_ratio_, feasibility_tolerance_; // physical limits and time adjustment ratio

  public:
    UniformBspline() {}
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
    ~UniformBspline();

    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    // initialize as an uniform B-spline
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // get / set basic bspline info

    void setKnot(const Eigen::VectorXd &knot);
    Eigen::VectorXd getKnot();
    Eigen::MatrixXd getControlPoint();
    double getInterval();
    bool getTimeSpan(double &um, double &um_p);

    // compute position / derivative

    Eigen::VectorXd evaluateDeBoor(const double &u);                                               // use u \in [up, u_mp]
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); } // use t \in [0, duration]
    UniformBspline getDerivative();

    // 3D B-spline interpolation of points in point_set, with boundary vel&acc
    // constraints.  Bounded single-QR parameterization (implementation_plan_
    // 26082722):
    //   - on entry ctrl_pts is cleared so a failed call never leaves a stale
    //     or partial output;
    //   - ts must be finite and > 0; point_set size must be within [4,
    //     max_points]; derivatives must be exactly 4; all values finite;
    //   - x/y/z are solved with ONE pivoted QR on the shared (K+4)x(K+2)
    //     system (single decomposition, 3-column RHS) and the result is
    //     assigned only after the solver succeeds and the solution is finite.
    // input : (K+2) points with boundary vel/acc; ts; max_points
    // output: (K+6) control_pts; returns true only after a complete success.
    static bool parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                      const vector<Eigen::Vector3d> &start_end_derivative,
                                      std::size_t max_points,
                                      Eigen::MatrixXd &ctrl_pts);

    /* check feasibility, adjust time */

    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
    bool checkFeasibility(double &ratio, bool show = false);
    void lengthenTime(const double &ratio);

    /* for performance evaluation */

    double getTimeSum();
    double getLength(const double &res = 0.01);
    double getJerk();
    void getMeanAndMaxVel(double &mean_v, double &max_v);
    void getMeanAndMaxAcc(double &mean_a, double &max_a);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace ego_planner
#endif