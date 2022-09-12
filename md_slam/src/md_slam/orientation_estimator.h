
#pragma once
#include <srrg_config/configurable.h>
#include <srrg_config/property_configurable.h>
#include <srrg_geometry/geometry3d.h>
#include <srrg_system_utils/shell_colors.h>

// approach taken from "Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and
// MARGs" https://www.mdpi.com/1424-8220/15/8/19302

namespace md_slam {

  using namespace srrg2_core;

  class MDOrientationEstimator : public srrg2_core::Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MDOrientationEstimator() {
    }

    PARAM(PropertyBool,
          estimate_bias,
          "if set to true estimate biases from gyroscope",
          true,
          nullptr);

    PARAM(PropertyBool,
          calculate_adaptive_gain,
          "if set to true calculate gain adaptively",
          true,
          nullptr);

    inline Matrix3d rotation() const {
      return _quat.toRotationMatrix();
    }

    inline const Quaterniond& quaternion() const {
      return _quat;
    }

    inline const Vector3d& biasGyroscope() const {
      return _bias_gyro;
    }

    inline void setRotation(const Matrix3d& rot_) {
      _quat = Quaterniond(rot_);
    }

    inline void reset() {
      _quat.setIdentity();
      _initialized = false;
    }

    inline void scaleQuaternion(Quaterniond& q_, const double& gain_) {
      if (q_.w() < 0.0) {
        q_.slerp(gain_, q_);
        // slerp (spherical linear interpolation)
        // const double angle = acos(q_.w());
        // const double sin_angle = sin(angle);
        // const double A     = sin(angle * (1.0 - gain_)) / sin_angle;
        // const double B     = sin(angle * gain_) / sin_angle;
        // q_.w()       = A + B * q_.w();
        // q_.x()       = B * q_.x();
        // q_.y()       = B * q_.y();
        // q_.z()       = B * q_.z();
      } else {
        // lerp (linear interpolation)
        q_.w() = (1.0 - gain_) + gain_ * q_.w();
        q_.x() = gain_ * q_.x();
        q_.y() = gain_ * q_.y();
        q_.z() = gain_ * q_.z();
      }
      q_.normalize();
    }

    inline void initialize(Quaterniond& quat_meas, const Vector3d& acc_) {
      // quat_meas is the quaternion obtained from the acceleration vector representing
      // the orientation of the Global frame wrt the Local frame with arbitrary yaw
      // (intermediary frame), quat_meas.z() is defined as 0
      const Vector3d n_acc = acc_.normalized();
      if (n_acc.z() >= 0) {
        quat_meas.w() = sqrt((n_acc.z() + 1) * 0.5);
        quat_meas.x() = -n_acc.y() / (2.0 * quat_meas.w());
        quat_meas.y() = n_acc.x() / (2.0 * quat_meas.w());
        quat_meas.z() = 0;
      } else {
        const double X = sqrt((1 - n_acc.z()) * 0.5);
        quat_meas.w()  = -n_acc.y() / (2.0 * X);
        quat_meas.x()  = X;
        quat_meas.y()  = 0;
        quat_meas.z()  = n_acc.x() / (2.0 * X);
      }
    }

    inline Quaterniond predict(const Vector3d& gyro_, const Quaterniond& q_, const double& dt_) {
      const Vector3d w_unb = gyro_ - _bias_gyro;
      const double half_dt = 0.5 * dt_;
      // clang-format off
      Quaterniond res;
      res.w() = q_.w() + half_dt * (w_unb.x() * q_.x() + w_unb.y() * q_.y() + w_unb.z() * q_.z());
      res.x() = q_.x() + half_dt * (-w_unb.x() * q_.w() - w_unb.y() * q_.z() + w_unb.z() * q_.y());
      res.y() = q_.y() + half_dt * (w_unb.x() * q_.z() - w_unb.y() * q_.w() - w_unb.z() * q_.x());
      res.z() = q_.z() + half_dt * (-w_unb.x() * q_.y() + w_unb.y() * q_.x() - w_unb.z() * q_.w());
      res.normalize();
      // clang-format on
      return res;
    }

    inline bool isSteady(const Vector3d& acc_, const Vector3d& gyro_) const {
      const double acc_magnitude = acc_.norm();
      if (fabs(acc_magnitude - _gravity) > _acc_threshold)
        return false;

      if (fabs(gyro_.x() - _gyro_prev.x()) > _delta_gyro_threshold ||
          fabs(gyro_.y() - _gyro_prev.y()) > _delta_gyro_threshold ||
          fabs(gyro_.z() - _gyro_prev.z()) > _delta_gyro_threshold)
        return false;

      if (fabs(gyro_.x() - _bias_gyro.x()) > _gyro_threshold ||
          fabs(gyro_.y() - _bias_gyro.y()) > _gyro_threshold ||
          fabs(gyro_.z() - _bias_gyro.z()) > _gyro_threshold)
        return false;

      std::cerr << FG_GREEN("steady state: updating bias gyroscope!") << std::endl;
      return true;
    }

    inline void updateBiases(const Vector3d& acc_, const Vector3d& gyro_) {
      _steady_state = isSteady(acc_, gyro_);
      if (_steady_state) { // if steady update gyro bias
        _bias_gyro += _bias_alpha * (gyro_ - _bias_gyro);
      }
      _gyro_prev = gyro_;
    }

    inline double adaptiveGain(const Vector3d& acc_, const double& alpha_) {
      const double acc_magnitude = acc_.norm();
      const double error         = fabs(acc_magnitude - _gravity) / _gravity;
      const double m             = 1.0 / (_adap_gain_thrs1 - _adap_gain_thrs2);
      const double b             = 1.0 - m * _adap_gain_thrs1;
      // init and get gain factor
      double factor = 0.0;
      if (error < _adap_gain_thrs1)
        factor = 1.0;
      else if (error < _adap_gain_thrs2)
        factor = m * error + b;
      else
        factor = 0.0;
      // printf("FACTOR: %f \n", factor);
      return factor * alpha_;
    }

    inline Quaterniond accCorrection(const Vector3d& acc_, const Quaterniond& q_) {
      // normalize acceleration vector
      const Vector3d acc_n = acc_.normalized();
      // acceleration reading rotated into the world frame by the inverse
      // predicted quaternion (predicted gravity)
      const Vector3d pred_g = q_.conjugate() * acc_n;
      // delta quaternion that rotates the predicted gravity into the real gravity
      Quaterniond dq;
      dq.w() = sqrt((pred_g.z() + 1) * 0.5);
      dq.x() = -pred_g.y() / (2.0 * dq.w());
      dq.y() = pred_g.x() / (2.0 * dq.w());
      dq.z() = 0.0;
      return dq;
    }

    inline void update(const Vector3d& acc_, const Vector3d gyro_, const double dt_) {
      if (!_initialized) {
        // first time - ignore prediction
        initialize(_quat, acc_);
        _initialized = true;
        return;
      }

      // estimate biases
      if (param_estimate_bias.value())
        updateBiases(acc_, gyro_);

      Quaterniond q_pred = predict(gyro_, _quat, dt_);

      // correction (from acc)
      // q_ = q_pred * [(1-gain) * qI + gain * dq_acc] where qI is identity quat
      Quaterniond dq_acc = accCorrection(acc_, q_pred);

      double gain = _gain_acc;
      if (param_calculate_adaptive_gain.value()) {
        gain = adaptiveGain(acc_, _gain_acc);
      }

      // spherical or linear interpolate
      scaleQuaternion(dq_acc, gain);

      _quat = q_pred * dq_acc;
      _quat.normalize();
      // integrate velocity and position here
    }

  protected:
    // const double _gravity = 9.80665; // [m/s^2]
    const double _gravity = 9.81; // [m/s^2]
    // bias estimation steady state thresholds
    const double _gyro_threshold       = 0.3;  //  [rad/s] TODO tune
    const double _acc_threshold        = 0.2;  //  [m/s^2] TODO tune
    const double _delta_gyro_threshold = 0.02; //  [rad/s] TODO tune

    // gain parameter for the filter, belongs in [0, 1]
    double _gain_acc = 0.01; // TODO tune

    // bias estimation gain parameter, belongs in [0, 1]
    double _bias_alpha = 0.01;

    // adaptive gain parameters
    double _adap_gain_thrs1 = 0.1;
    double _adap_gain_thrs2 = 0.2;

    // important flags
    bool _initialized  = false;
    bool _steady_state = false;

    // represents the orientation of the fixed frame wrt the body frame
    Quaterniond _quat = Quaterniond::Identity();
    // keep prev gyro to check steady state
    Vector3d _gyro_prev = Vector3d::Zero();
    // bias in angular velocities
    Vector3d _bias_gyro = Vector3d::Zero();
  };

} // namespace md_slam
