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
