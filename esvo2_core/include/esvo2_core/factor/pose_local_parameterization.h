#ifndef ESVO2_CORE_FACTOR_POSELOCAL_H
#define ESVO2_CORE_FACTOR_POSELOCAL_H

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <esvo2_core/factor/utility.h>
namespace esvo2_core {
namespace factor {
class PoseLocalParameterization : public ceres::Manifold {
public:
  bool Plus(const double *x, const double *delta,
            double *x_plus_delta) const override {
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq =
        Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
  }

  bool PlusJacobian(const double *x, double *jacobian) const override {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
  }

  bool Minus(const double *y, const double *x,
             double *y_minus_x) const override {
    Eigen::Map<const Eigen::Vector3d> p_x(x);
    Eigen::Map<const Eigen::Vector3d> p_y(y);
    Eigen::Map<const Eigen::Quaterniond> q_x(x + 3);
    Eigen::Map<const Eigen::Quaterniond> q_y(y + 3);
    Eigen::Map<Eigen::Vector3d> dp(y_minus_x);
    Eigen::Map<Eigen::Vector3d> dq(y_minus_x + 3);

    dp = p_y - p_x;
    const Eigen::Quaterniond delta_q = q_x.conjugate() * q_y;
    dq = 2.0 * delta_q.vec();

    return true;
  }

  bool MinusJacobian(const double *x, double *jacobian) const override {
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobian);
    j.setZero();
    j.topLeftCorner<3, 3>().setIdentity();
    j.bottomRightCorner<3, 3>().setIdentity();

    return true;
  }

  int AmbientSize() const override { return 7; }
  int TangentSize() const override { return 6; }
};
} // namespace factor
} // namespace esvo2_core

#endif
