// Copyright (c) 2026 Rhombus Systems
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OPENNAV_DOCKING__DOCKING_ALIGN_LAW_HPP_
#define OPENNAV_DOCKING__DOCKING_ALIGN_LAW_HPP_

// Pure-math helpers for the docking pre-alignment stage and the contact yaw
// gate. Header-only with no ROS or generated-message dependencies so the
// native x86 unit-test harness (rhombus_ros2_agent) can exercise the docking
// geometry directly.

#include <algorithm>
#include <cmath>

namespace opennav_docking
{

/// Normalize an angle to (-pi, pi].
inline double normalizeAngle(double a)
{
  const double two_pi = 2.0 * M_PI;
  a = std::fmod(a, two_pi);
  if (a <= -M_PI) {
    a += two_pi;
  } else if (a > M_PI) {
    a -= two_pi;
  }
  return a;
}

/// Signed perpendicular distance of the robot from the dock axis, computed in
/// the robot base frame. The dock is at (x_d, y_d) and its axis runs through
/// that point along heading yaw_d (the direction of travel at contact).
/// Positive = the robot sits to the LEFT of the axis (so it must strafe right,
/// i.e. negative base-frame vy, to reach it).
inline double lateralOffsetFromDockAxis(double x_d, double y_d, double yaw_d)
{
  return x_d * std::sin(yaw_d) - y_d * std::cos(yaw_d);
}

struct StrafeParams
{
  double k_lateral = 1.5;        // proportional gain on the lateral offset
  double v_lateral_min = 0.0;    // gait floor: minimum magnitude when nonzero
  double v_lateral_max = 0.15;   // cap; <= 0 disables strafing entirely
  double lateral_deadband = 0.02;  // no command inside this offset
};

/// Base-frame vy command that drives the lateral offset to zero. Returns 0
/// inside the deadband or when strafing is disabled (v_lateral_max <= 0).
inline double computeStrafeCommand(double e_lat, const StrafeParams & p)
{
  if (p.v_lateral_max <= 0.0 || std::fabs(e_lat) < p.lateral_deadband) {
    return 0.0;
  }
  double magnitude = std::min(p.k_lateral * std::fabs(e_lat), p.v_lateral_max);
  if (p.v_lateral_min > 0.0) {
    magnitude = std::max(magnitude, p.v_lateral_min);
  }
  // Positive offset = robot left of the axis = strafe right (negative vy).
  return e_lat > 0.0 ? -magnitude : magnitude;
}

/// Platform gait floor for the pre-alignment rotate command. Quadrupeds
/// (Go2) do not take a step below a minimum in-place yaw rate, so a command
/// under the floor produces zero motion and the bearing never converges
/// (Pozole 2026-09-01: 0.15 rad/s commanded for 15 s, dog never moved).
/// The floor applies only while the bearing is still outside tolerance —
/// once in-band the raw command passes through so the robot is not forced to
/// micro-twitch. Sign is preserved; min_angular_vel <= 0 disables the floor.
///
/// The floor is also skipped while the command's sign disagrees with the
/// bearing's. That is the accel ramp decelerating through zero after the
/// bearing changed side; flooring it re-asserts the OLD direction at full
/// gait speed every tick, the ramp (anchored to the last command) can then
/// never cross zero, and the robot spins the wrong way until the tag leaves
/// the FOV (Pozole 2026-09-01 15:05:23: bearing +0.60 rad, wz -0.400 for 3 s).
/// Passing the ramp through lets it cross within a few ticks, after which the
/// floor engages in the right direction.
inline double applyRotationFloor(
  double wz, double bearing, double bearing_tolerance, double min_angular_vel)
{
  if (min_angular_vel <= 0.0 || wz == 0.0 || std::fabs(bearing) < bearing_tolerance) {
    return wz;
  }
  if ((wz > 0.0) != (bearing > 0.0)) {
    return wz;
  }
  const double magnitude = std::max(std::fabs(wz), min_angular_vel);
  return wz > 0.0 ? magnitude : -magnitude;
}

struct BearingHoldParams
{
  double tolerance = 0.10;          // engage the hold at |bearing| >= tolerance
  double k_bearing = 1.5;           // proportional gain once engaged
  double min_angular_vel = 0.0;     // gait floor: minimum magnitude when engaged
  double max_angular_vel = 0.5;     // cap on the hold command
};

/// Stage-2 bearing hold for the pre-alignment strafe: keep the dock centred
/// while strafing without running the full rotate-to-heading controller.
/// That controller drives at its cruise rate toward zero bearing and, with
/// detection/filter lag, overshoots a 4 deg error into a 30 deg one (Pozole
/// 2026-09-01 15:05:22: wz -0.41 for bearing -0.073, next sample +0.60).
/// Bang-bang with hysteresis instead: engage at |bearing| >= tolerance,
/// release below tolerance/2, output sign(bearing) * clamp(k*|bearing|,
/// min, max) while engaged and 0 otherwise. hold_active carries the
/// hysteresis state between ticks.
inline double computeBearingHoldCommand(
  double bearing, const BearingHoldParams & p, bool & hold_active)
{
  const double abs_bearing = std::fabs(bearing);
  if (abs_bearing >= p.tolerance) {
    hold_active = true;
  } else if (abs_bearing < 0.5 * p.tolerance) {
    hold_active = false;
  }
  if (!hold_active || bearing == 0.0) {
    return 0.0;
  }
  double magnitude = std::min(p.k_bearing * abs_bearing, p.max_angular_vel);
  if (p.min_angular_vel > 0.0) {
    magnitude = std::max(magnitude, p.min_angular_vel);
  }
  return bearing > 0.0 ? magnitude : -magnitude;
}

/// True when the robot sits at or behind the staging plane — the plane
/// through the staging pose perpendicular to its heading (staging yaw points
/// at the dock, so positive longitudinal displacement means the robot is on
/// the dock side and still needs to back out). dx/dy are robot position
/// minus staging position in the same fixed frame.
inline bool isBehindStagingPlane(double dx, double dy, double staging_yaw)
{
  return dx * std::cos(staging_yaw) + dy * std::sin(staging_yaw) <= 0.0;
}

/// Contact criterion: XY distance within dist_thresh AND, when yaw_thresh > 0,
/// heading error within yaw_thresh. yaw_thresh <= 0 preserves the historical
/// XY-only behavior. yaw_err may be un-normalized; it is wrapped here.
inline bool isWithinContactTolerance(
  double dist, double yaw_err, double dist_thresh, double yaw_thresh)
{
  if (dist >= dist_thresh) {
    return false;
  }
  if (yaw_thresh <= 0.0) {
    return true;
  }
  return std::fabs(normalizeAngle(yaw_err)) < yaw_thresh;
}

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__DOCKING_ALIGN_LAW_HPP_
