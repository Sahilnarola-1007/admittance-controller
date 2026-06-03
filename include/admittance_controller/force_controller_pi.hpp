#ifndef ADMITTANCE_CONTROLLER_FORCE_CONTROLLER_PI_HPP
#define ADMITTANCE_CONTROLLER_FORCE_CONTROLLER_PI_HPP

#include <algorithm>   // std::clamp

namespace admittance {

// ============================================================
// ForceControllerPI — single-axis PI force regulator.
//
// SIGN CONVENTION (Z-axis, base frame):
//   Hardware reality (wrench_corrected):
//     Press UP (into surface / contact direction)  →  Fz NEGATIVE
//     Pull DOWN (away from surface)                →  Fz POSITIVE
//
//   The admittance node calls:
//     vz = force_pi_->update(F_desired, -fz)
//
//   The negation flips the sign so the PI sees:
//     F_desired  > 0   : desired contact force magnitude (e.g. +5 N)
//     F_measured > 0   : contact force after sign flip (-(-5) = +5)
//     e = F_des - F_meas > 0  →  need MORE force
//     v_unsat = -(kp·e + ki·∫e)  →  negative = move DOWN into surface
//
//   Anti-windup: conditional integration freezes the integral when
//   the output is saturated AND the error would make it worse.
//   This prevents the -10 N overshoot spike seen at wipe direction
//   reversals with aggressive Ki.
//
// Validated gains (surface wiping demo, June 2026):
//   Kp = 0.001, Ki = 0.01  →  steady-state tracking [-5.5, -4.5] N
//   for F_desired = 5 N. Overshoot envelope: -7 N max, -3 N min.
// ============================================================
class ForceControllerPI {
public:
  ForceControllerPI(double kp, double ki, double dt, double v_max)
    : kp_(kp),ki_(ki),dt_(dt),v_max_(v_max){}

  // One control step. Returns saturated velocity command [m/s].
    double update(double force_desired, double force_measured) {
        double e       = force_desired - force_measured;
        double i_cand  = integral_ + e * dt_;
        double v_unsat = -(kp_ * e + ki_ * i_cand);
        double v_sat   = std::clamp(v_unsat, -v_max_, +v_max_);

        bool saturated = (v_unsat != v_sat);
        if (!saturated || sameSign(e, v_sat)) {
            integral_ = i_cand;   // Add integral terms when needed
        }
        return v_sat;
    }

  // Bumpless transfer: call on entering WIPE.
  void reset(double integral_init = 0.0) {
      integral_ = integral_init;
  }

private:
  // helper for the anti-windup guard
  static bool sameSign(double a, double b) {
     return (a >= 0.0) == (b >= 0.0);
  }

  double kp_, ki_, dt_, v_max_;
  double integral_ = 0.0;
};

}  // namespace admittance

#endif