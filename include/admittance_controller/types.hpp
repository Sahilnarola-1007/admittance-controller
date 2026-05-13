#pragma once

#include <cmath>
#include <algorithm>

namespace admittance {

// ============================================================================
// Wrench: the 6D force/torque vector (Fx, Fy, Fz, Tx, Ty, Tz)
// ============================================================================
struct Wrench {
  double fx = 0.0, fy = 0.0, fz = 0.0;  // Forces (N)
  double tx = 0.0, ty = 0.0, tz = 0.0;  // Torques (Nm)

  Wrench operator-(const Wrench& other) const {
    return {fx - other.fx, fy - other.fy, fz - other.fz,
            tx - other.tx, ty - other.ty, tz - other.tz};
  }

  Wrench operator+(const Wrench& other) const {
    return {fx + other.fx, fy + other.fy, fz + other.fz,
            tx + other.tx, ty + other.ty, tz + other.tz};
  }

  Wrench operator/(double s) const {
    return {fx/s, fy/s, fz/s, tx/s, ty/s, tz/s};
  }

  Wrench& operator+=(const Wrench& other) {
    fx += other.fx; fy += other.fy; fz += other.fz;
    tx += other.tx; ty += other.ty; tz += other.tz;
    return *this;
  }

  double force_magnitude() const {
    return std::sqrt(fx*fx + fy*fy + fz*fz);
  }
};

// ============================================================================
// Cartesian pose (position only — orientation stays fixed in v1)
// ============================================================================
struct CartesianPos {
  double x = 0.0, y = 0.0, z = 0.0;

  CartesianPos operator+(const CartesianPos& other) const {
    return {x + other.x, y + other.y, z + other.z};
  }

  CartesianPos operator-(const CartesianPos& other) const {
    return {x - other.x, y - other.y, z - other.z};
  }
};

// ============================================================================
// Exponential Moving Average filter for wrench data
//
// filtered += alpha * (raw - filtered)
//
// alpha = 0.1 → heavy smoothing, laggy but stable
// alpha = 0.5 → light smoothing, responsive but noisier
// ============================================================================
class WrenchFilter {
public:
  explicit WrenchFilter(double alpha = 0.1) : alpha_(alpha) {}

  void set_alpha(double alpha) {
    alpha_ = std::clamp(alpha, 0.01, 1.0);
  }

  Wrench update(const Wrench& raw) {
    filtered_.fx += alpha_ * (raw.fx - filtered_.fx);
    filtered_.fy += alpha_ * (raw.fy - filtered_.fy);
    filtered_.fz += alpha_ * (raw.fz - filtered_.fz);
    filtered_.tx += alpha_ * (raw.tx - filtered_.tx);
    filtered_.ty += alpha_ * (raw.ty - filtered_.ty);
    filtered_.tz += alpha_ * (raw.tz - filtered_.tz);
    return filtered_;
  }

  void reset() { filtered_ = Wrench{}; }
  Wrench current() const { return filtered_; }

private:
  double alpha_;
  Wrench filtered_;
};

// ============================================================================
// Dead zone: forces below threshold are treated as zero
//
// Also subtracts the threshold so movement starts smoothly from zero
// instead of jumping when you cross the threshold.
// ============================================================================
inline double apply_deadzone(double value, double threshold) {
  if (std::abs(value) < threshold) return 0.0;
  return (value > 0.0) ? (value - threshold) : (value + threshold);
}

} // namespace admittance
