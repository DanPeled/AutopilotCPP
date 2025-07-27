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
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>

#include <optional>

#include "profile.h"
#include "target.h"

namespace autopilot {
struct APResult {
  units::meters_per_second_t vx;
  units::meters_per_second_t vy;
  frc::Rotation2d targetAngle;
};

/**
 * Autopilot is a class that tries to drive a target to a goal in 2 dimensional
 * space.
 *
 * Autopilot is a fast algorithm because it doesn not think ahead. Any and all
 * math is already worked out such that only a small amount of computation is
 * necessary on the fly.
 *
 *
 * This means that autopilot is un able to avoid obstacles, because it cannot
 * think ahead.
 */
class Autopilot {
 public:
  Autopilot() = delete;

  /**
   * Constructs an Autopilot from a given profile. This is the profile that the
   * autopilot will use for all actions.
   */
  explicit Autopilot(const APProfile& profile);

  /**
   * Returns the next field relative velocity for the trajectory
   *
   * @param current The robot's current position.
   * @param velocity The robot's current <b>field relative</b> velocity.
   * @param target The target the robot should drive towards.
   */
  APResult Calculate(const frc::Pose2d& current,
                     const frc::Translation2d& velocity,
                     const APTarget& target);
  /**
   * Returns whether the given pose is within tolerance for the target
   */
  bool AtTarget(const frc::Pose2d& current, const APTarget& target);

 private:
  APProfile m_profile;
  static constexpr units::second_t dt = 20_ms;

  /**
   * Turns any other coordinate frame into a coordinate frame with positive x
   * meaning in the direction of the target's entry angle, if applicable
   * (otherwise no change to angles).
   */
  frc::Translation2d ToTargetCoordinateFrame(const frc::Translation2d& coords,
                                             const APTarget& target);
  /**
   * Turns a translation from a target-relative coordinate frame to a global
   * coordinate frame.
   */
  frc::Translation2d ToGlobalCoordinateFrame(const frc::Translation2d& coords,
                                             const APTarget& target);
  /**
   * Determines the maximum velocity required to travel the given distance and
   * end at the desired end velocity.
   */
  units::meters_per_second_t CalculateMaxVelocity(
      units::meter_t dist, units::meters_per_second_t endVelo);
  /**
   * Attempts to drive the initial translation to the goal translation using the
   * parameters for acceleration given in the profile.
   */
  frc::Translation2d Correct(const frc::Translation2d& initial,
                             const frc::Translation2d& goal);
  /**
   * Using the provided acceleration, "pushes" the start point towards the end
   * point.
   *
   * This is used for ensuring that changes in velocity are withing the
   * acceleration threshold
   */
  double Push(double start, double end,
              units::meters_per_second_squared_t accel);
  /**
   * Uses the swirly method to calculate the correct velocities for the robot,
   * respecting entry angles
   *
   * @param offset The offset from the robot to the target, in the target's
   * coordinate frame
   */
  frc::Translation2d CalculateSwirlyVelocity(const frc::Translation2d& offset,
                                             const APTarget& target);
  /**
   * Using a precomputed integral, returns the length of the path that the
   * swirly method generates.
   *
   * More specificallu, this calcualtes the arc length of the polar curve
   * r=theta from the given angle to zero, then scales it to match the current
   * state.
   */
  units::meter_t CalculateSwirlyLength(units::radian_t theta,
                                       units::meter_t radius);
  /**
   * Returns the correct target heading for the current state
   */
  frc::Rotation2d GetRotationTarget(const frc::Rotation2d& current,
                                    const APTarget& target,
                                    units::meter_t dist);
};
}  // namespace autopilot
