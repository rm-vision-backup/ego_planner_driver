#include "ego_planner_driver/uniform_bspline.h"
// removed: no ROS dependency in extracted kernel

#include <cmath>

namespace ego_planner
{

  UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order,
                                 const double &interval)
  {
    setUniformBspline(points, order, interval);
  }

  UniformBspline::~UniformBspline() {}

  void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order,
                                         const double &interval)
  {
    control_points_ = points;
    p_ = order;
    interval_ = interval;

    n_ = points.cols() - 1;
    m_ = n_ + p_ + 1;

    u_ = Eigen::VectorXd::Zero(m_ + 1);
    for (int i = 0; i <= m_; ++i)
    {

      if (i <= p_)
      {
        u_(i) = double(-p_ + i) * interval_;
      }
      else if (i > p_ && i <= m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
      else if (i > m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
    }
  }

  void UniformBspline::setKnot(const Eigen::VectorXd &knot) { this->u_ = knot; }

  Eigen::VectorXd UniformBspline::getKnot() { return this->u_; }

  bool UniformBspline::getTimeSpan(double &um, double &um_p)
  {
    if (p_ > u_.rows() || m_ - p_ > u_.rows())
      return false;

    um = u_(p_);
    um_p = u_(m_ - p_);

    return true;
  }

  Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }

  Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u)
  {

    double ub = min(max(u_(p_), u), u_(m_ - p_));

    // determine which [ui,ui+1] lay in
    int k = p_;
    while (true)
    {
      if (u_(k + 1) >= ub)
        break;
      ++k;
    }

    /* deBoor's alg */
    vector<Eigen::VectorXd> d;
    for (int i = 0; i <= p_; ++i)
    {
      d.push_back(control_points_.col(k - p_ + i));
      // cout << d[i].transpose() << endl;
    }

    for (int r = 1; r <= p_; ++r)
    {
      for (int i = p_; i >= r; --i)
      {
        double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
        // cout << "alpha: " << alpha << endl;
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
      }
    }

    return d[p_];
  }

  // Eigen::VectorXd UniformBspline::evaluateDeBoorT(const double& t) {
  //   return evaluateDeBoor(t + u_(p_));
  // }

  Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
  {
    // The derivative of a b-spline is also a b-spline, its order become p_-1
    // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
    Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
    for (int i = 0; i < ctp.cols(); ++i)
    {
      ctp.col(i) =
          p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
    }
    return ctp;
  }

  UniformBspline UniformBspline::getDerivative()
  {
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    UniformBspline derivative(ctp, p_ - 1, interval_);

    /* cut the first and last knot */
    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);
    derivative.setKnot(knot);

    return derivative;
  }

  double UniformBspline::getInterval() { return interval_; }

  void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
  {
    limit_vel_ = vel;
    limit_acc_ = acc;
    limit_ratio_ = 1.1;
    feasibility_tolerance_ = tolerance;
  }

  bool UniformBspline::checkFeasibility(double &ratio, bool show)
  {
    bool fea = true;

    Eigen::MatrixXd P = control_points_;
    int dimension = control_points_.rows();

    /* check vel feasibility and insert points */
    double max_vel = -1.0;
    double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 1; ++i)
    {
      Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1));

      if (fabs(vel(0)) > enlarged_vel_lim || fabs(vel(1)) > enlarged_vel_lim ||
          fabs(vel(2)) > enlarged_vel_lim)
      {

        if (show)
          cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
        fea = false;

        for (int j = 0; j < dimension; ++j)
        {
          max_vel = max(max_vel, fabs(vel(j)));
        }
      }
    }

    /* acc feasibility */
    double max_acc = -1.0;
    double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 2; ++i)
    {

      Eigen::VectorXd acc = p_ * (p_ - 1) *
                            ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                             (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                            (u_(i + p_ + 1) - u_(i + 2));

      if (fabs(acc(0)) > enlarged_acc_lim || fabs(acc(1)) > enlarged_acc_lim ||
          fabs(acc(2)) > enlarged_acc_lim)
      {

        if (show)
          cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
        fea = false;

        for (int j = 0; j < dimension; ++j)
        {
          max_acc = max(max_acc, fabs(acc(j)));
        }
      }
    }

    ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

    return fea;
  }

  void UniformBspline::lengthenTime(const double &ratio)
  {
    int num1 = 5;
    int num2 = getKnot().rows() - 1 - 5;

    double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
    double t_inc = delta_t / double(num2 - num1);
    for (int i = num1 + 1; i <= num2; ++i)
      u_(i) += double(i - num1) * t_inc;
    for (int i = num2 + 1; i < u_.rows(); ++i)
      u_(i) += delta_t;
  }

  // void UniformBspline::recomputeInit() {}

  bool UniformBspline::parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                             const vector<Eigen::Vector3d> &start_end_derivative,
                                             std::size_t max_points,
                                             Eigen::MatrixXd &ctrl_pts)
  {
    // 入口立即清空输出：失败/成功均不残留旧或半成品控制点。
    ctrl_pts.resize(0, 0);

    if (!(std::isfinite(ts)) || ts <= 0.0)
    {
      cout << "[B-spline]:time step error." << endl;
      return false;
    }

    if (point_set.size() <= 3)
    {
      cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
      return false;
    }

    if (max_points < 4 || point_set.size() > max_points)
    {
      cout << "[B-spline]:point set size " << point_set.size()
           << " outside [4, " << max_points << "]." << endl;
      return false;
    }

    if (start_end_derivative.size() != 4)
    {
      cout << "[B-spline]:derivatives error." << endl;
      return false;
    }

    // size_t -> Eigen::Index/int conversions are safe after the max_points cap.
    const Eigen::Index K = static_cast<Eigen::Index>(point_set.size());
    for (Eigen::Index i = 0; i < K; ++i)
    {
      if (!point_set[static_cast<size_t>(i)].allFinite())
      {
        cout << "[B-spline]:non-finite point in point_set." << endl;
        return false;
      }
    }
    for (size_t i = 0; i < start_end_derivative.size(); ++i)
    {
      if (!start_end_derivative[i].allFinite())
      {
        cout << "[B-spline]:non-finite boundary derivative." << endl;
        return false;
      }
    }

    // write A (single shared system, once)
    Eigen::Vector3d prow(3), vrow(3), arow(3);
    prow << 1, 4, 1;
    vrow << -1, 0, 1;
    arow << 1, -2, 1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

    for (Eigen::Index i = 0; i < K; ++i)
      A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

    A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

    A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
    A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();

    // write b: (K+4) x 3, x/y/z in one RHS
    Eigen::MatrixXd B(K + 4, 3);
    for (Eigen::Index i = 0; i < K; ++i)
    {
      B(i, 0) = point_set[static_cast<size_t>(i)](0);
      B(i, 1) = point_set[static_cast<size_t>(i)](1);
      B(i, 2) = point_set[static_cast<size_t>(i)](2);
    }
    for (int i = 0; i < 4; ++i)
    {
      B(K + i, 0) = start_end_derivative[static_cast<size_t>(i)](0);
      B(K + i, 1) = start_end_derivative[static_cast<size_t>(i)](1);
      B(K + i, 2) = start_end_derivative[static_cast<size_t>(i)](2);
    }

    // single pivoted QR decomposition, 3-column solve (bounded dense QR:
    // implementation_plan_26082722 §8; strict 256 default cap keeps the dense
    // case small. SparseQR/banded solver is a separate optimization.)
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
    const Eigen::MatrixXd solution = qr.solve(B);

    if (qr.info() != Eigen::Success)
      return false;
    if (!solution.allFinite())
      return false;
    // rank check: the solve reports rank; a rank-deficient system means the
    // point set cannot determine the boundary-constrained spline.
    if (qr.rank() < A.cols())
    {
      cout << "[B-spline]:rank-deficient parameterization." << endl;
      return false;
    }
    // residual: verify the over-determined LS system did not diverge.  The
    // (K+4)x(K+2) system carries boundary-velocity rows that a long route
    // cannot satisfy exactly, so the least-squares residual is non-zero by
    // design (§8.2).  A solve only "diverges" when the residual is comparable
    // to the system scale ||A||*||solution|| (+||b||); any normal segment
    // residual is orders of magnitude below that.  This rejects numerically
    // broken solves (huge non-finite coordinates) without rejecting the 60 m
    // regression path or normal short segments (§13.2).
    const Eigen::MatrixXd residual = A * solution - B;
    const double system_scale =
        A.lpNorm<Eigen::Infinity>() * (solution.norm() + 1.0) + B.norm() + 1.0;
    if (!std::isfinite(system_scale) || residual.norm() > 1e-3 * system_scale)
    {
      cout << "[B-spline]:residual too large." << endl;
      return false;
    }

    // convert to control pts only after the complete solve succeeded.
    // solution is (K+2) x 3; transposed here into (3 x K+2).
    ctrl_pts = solution.transpose();

    // cout << "[B-spline]: parameterization ok." << endl;
    return true;
  }

  double UniformBspline::getTimeSum()
  {
    double tm, tmp;
    if (getTimeSpan(tm, tmp))
      return tmp - tm;
    else
      return -1.0;
  }

  double UniformBspline::getLength(const double &res)
  {
    double length = 0.0;
    double dur = getTimeSum();
    Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
    for (double t = res; t <= dur + 1e-4; t += res)
    {
      p_n = evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  double UniformBspline::getJerk()
  {
    UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

    Eigen::VectorXd times = jerk_traj.getKnot();
    Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
    int dimension = ctrl_pts.rows();

    double jerk = 0.0;
    for (int i = 0; i < ctrl_pts.cols(); ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
      }
    }

    return jerk;
  }

  void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    UniformBspline vel = getDerivative();
    double tm, tmp;
    vel.getTimeSpan(tm, tmp);

    double max_vel = -1.0, mean_vel = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
      double vn = vxd.norm();

      mean_vel += vn;
      ++num;
      if (vn > max_vel)
      {
        max_vel = vn;
      }
    }

    mean_vel = mean_vel / double(num);
    mean_v = mean_vel;
    max_v = max_vel;
  }

  void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    UniformBspline acc = getDerivative().getDerivative();
    double tm, tmp;
    acc.getTimeSpan(tm, tmp);

    double max_acc = -1.0, mean_acc = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd axd = acc.evaluateDeBoor(t);
      double an = axd.norm();

      mean_acc += an;
      ++num;
      if (an > max_acc)
      {
        max_acc = an;
      }
    }

    mean_acc = mean_acc / double(num);
    mean_a = mean_acc;
    max_a = max_acc;
  }
} // namespace ego_planner
