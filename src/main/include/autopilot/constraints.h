// Copyright (c) 2025 Dan Peled
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
//
// Inspired by the Autopilot project in Java:
// https://github.com/therekrab/autopilot/

#pragma once

#include <units/acceleration.h>
#include <units/velocity.h>

namespace autopilot {
/**
 * A class that holds constraint information for an autopilot action.
 *
 * Constraints are max velocity, acceleration, and jerk.
 */
class APConstraints {
 public:
  /** Creates a blank APConstraints with no limit on velocity */
  APConstraints();

  /**
   * Creates a new APConstraints with given max velocity, acceleration, and
   * jerk.
   */
  APConstraints(units::meters_per_second_t velocity,
                units::meters_per_second_squared_t acceleration, double jerk);

  /**
   * Creates a new APConstraints with a given max acceleration and jerk.
   * Velocity is left unlimited.
   */
  APConstraints(units::meters_per_second_squared_t acceleration, double jerk);

  /**
   * Modifies this constraint's max velocity and returns itself.
   */
  APConstraints& withVelocity(units::meters_per_second_t newVelocity);

  /**
   * Modifies this constraint's max acceleration value and returns itself.
   */
  APConstraints& withAcceleration(
      units::meters_per_second_squared_t newAcceleration);

  /**
   * Modifies this constraint's max jerk value and returns itself.
   */
  APConstraints& withJerk(double newJerk);

  units::meters_per_second_t velocity;
  units::meters_per_second_squared_t acceleration;
  double jerk;  // Linear jerk in m/s^3
};
}  // namespace autopilot
