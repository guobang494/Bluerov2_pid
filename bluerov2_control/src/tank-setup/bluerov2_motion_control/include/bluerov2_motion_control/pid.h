/*
    The header of a PID control with antiwindup and saturation clamping. 
    author: Davide Grande
    date: 11/02/2026
*/

#pragma once  // this is an alternative to classical syntax with ifndef, endif 

#include <algorithm>
#include <cmath>

class SimplePID {
public:
  SimplePID() = default;

  void setGains(double kp, double ki, double kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
  }

  void setSaturation(double umin, double umax) {
    umin_ = umin; umax_ = umax;
  }

  void setAntiWindup(bool enabled) {
    antiwindup_ = enabled;
  }

  void setAngleWrap(bool enabled) {
    angle_wrap_ = enabled;
  }

  void reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    first_ = true;
  }

  double update(double ref, double meas, double dt) {
    if (dt <= 0.0) return last_u_;

    double error = ref - meas;
    if (angle_wrap_) {
      // Keep angular error in [-pi, pi] to avoid discontinuities at wrap-around.
      error = std::atan2(std::sin(error), std::cos(error));
    }

    double derivative = 0.0;
    if (!first_) {    
        derivative = (error - prev_error_) / dt;
    }
    first_ = false;

    // Integrate
    integral_ += error * dt;

    // Raw PID
    double u = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // Saturate
    double u_sat = clamp(u, umin_, umax_);

    // VAnti-windup: if saturated, undo last integration step
    if (antiwindup_ && u != u_sat) {
      integral_ -= error * dt; // revert integration when saturated
      u = kp_ * error + ki_ * integral_ + kd_ * derivative;
      u_sat = clamp(u, umin_, umax_);
    }

    prev_error_ = error;
    last_u_ = u_sat;
    return u_sat;
  }

private:
  static double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(x, hi));
  }

  double kp_{0.0}, ki_{0.0}, kd_{0.0};
  double umin_{-1e9}, umax_{1e9};
  bool antiwindup_{true};
  bool angle_wrap_{false};

  double integral_{0.0};
  double prev_error_{0.0};
  bool first_{true};

  double last_u_{0.0};
};
