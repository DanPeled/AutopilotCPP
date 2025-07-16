// Copyright (c) 2025 Dan Peled
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
//
// Inspired by the Autopilot project in Java:
// https://github.com/therekrab/autopilot/

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>

#include <optional>

#include "profile.h"
#include "target.h"

namespace autopilot {
class Autopilot {
 public:
  explicit Autopilot(const APProfile& profile);

  frc::Transform2d Calculate(const frc::Pose2d& current,
                             const frc::Translation2d& velocity,
                             const APTarget& target);

  bool AtTarget(const frc::Pose2d& current, const APTarget& target);

 private:
  APProfile m_profile;
  static constexpr units::second_t dt = 20_ms;

  frc::Translation2d ToTargetCoordinateFrame(const frc::Translation2d& coords,
                                             const APTarget& target);
  frc::Translation2d ToGlobalCoordinateFrame(const frc::Translation2d& coords,
                                             const APTarget& target);

  units::meters_per_second_t CalculateMaxVelocity(
      units::meter_t dist, units::meters_per_second_t endVelo);
  frc::Translation2d Correct(const frc::Translation2d& initial,
                             const frc::Translation2d& goal);
  double Push(double start, double end,
              units::meters_per_second_squared_t accel);
  frc::Translation2d CalculateSwirlyVelocity(const frc::Translation2d& offset,
                                             const APTarget& target);
  units::meter_t CalculateSwirlyLength(units::radian_t theta,
                                       units::meter_t radius);
  frc::Rotation2d GetRotationTarget(const frc::Rotation2d& current,
                                    const APTarget& target,
                                    units::meter_t dist);
};
}  // namespace autopilot
