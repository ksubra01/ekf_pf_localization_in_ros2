#pragma once

#include "ekf_pf_localization/common.hpp"

namespace ekf_pf_localization
{

class EKF
{
public:
  EKF()
  {
    x_.setZero();
    P_.setIdentity();
    Q_.setIdentity();
    R_.setIdentity();
  }

  void initialize(const State & initial_state)
  {
    x_ = stateToVector(initial_state);
  }

  void setProcessNoise(const Eigen::Matrix3d & Q)
  {
    Q_ = Q;
  }

  void setMeasurementNoise(const Eigen::Matrix2d & R)
  {
    R_ = R;
  }

  // ----------------------------
  // Prediction Step
  // ----------------------------
  void predict(const Control & u, double dt)
  {
    double theta = x_(2);

    // State prediction
    x_(0) += u.v * std::cos(theta) * dt;
    x_(1) += u.v * std::sin(theta) * dt;
    x_(2) += u.omega * dt;
    x_(2) = normalizeAngle(x_(2));

    // Jacobian of motion model
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0,2) = -u.v * std::sin(theta) * dt;
    F(1,2) =  u.v * std::cos(theta) * dt;

    // Covariance prediction
    P_ = F * P_ * F.transpose() + Q_;
  }

  // ----------------------------
  // Measurement Update
  // z = [x, y]
  // ----------------------------
  void update(const Eigen::Vector2d & z)
  {
    Eigen::Vector2d z_hat;
    z_hat << x_(0), x_(1);

    Eigen::Matrix<double, 2, 3> H;
    H << 1, 0, 0,
         0, 1, 0;

    Eigen::Vector2d y = z - z_hat;
    Eigen::Matrix2d S = H * P_ * H.transpose() + R_;
    Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;
  }

  // ----------------------------
  // Accessors
  // ----------------------------
  State getState() const
  {
    return vectorToState(x_);
  }

  Eigen::Matrix3d getCovariance() const
  {
    return P_;
  }

private:
  Eigen::Vector3d x_;   // state mean
  Eigen::Matrix3d P_;   // covariance
  Eigen::Matrix3d Q_;   // process noise
  Eigen::Matrix2d R_;   // measurement noise
};

}  // namespace ekf_pf_localization
