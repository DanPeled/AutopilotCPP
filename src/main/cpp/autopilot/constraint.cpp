// Copyright (c) 2025 Dan Peled
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
//
// Inspired by the Autopilot project in Java:
// https://github.com/therekrab/autopilot/

#include <limits>

#include "autopilot/constraints.h"

using namespace autopilot;

APConstraints::APConstraints()
    : velocity(units::meters_per_second_t{std::numeric_limits<double>::max()}),
      acceleration(units::meters_per_second_squared_t{0}),
      jerk(0.0) {}

APConstraints::APConstraints(units::meters_per_second_t velocity,
                             units::meters_per_second_squared_t acceleration,
                             double jerk)
    : velocity(velocity), acceleration(acceleration), jerk(jerk) {}

APConstraints::APConstraints(units::meters_per_second_squared_t acceleration,
                             double jerk)
    : velocity(units::meters_per_second_t{std::numeric_limits<double>::max()}),
      acceleration(acceleration),
      jerk(jerk) {}

APConstraints& APConstraints::withVelocity(
    units::meters_per_second_t newVelocity) {
  velocity = newVelocity;
  return *this;
}

APConstraints& APConstraints::withAcceleration(
    units::meters_per_second_squared_t newAcceleration) {
  acceleration = newAcceleration;
  return *this;
}

APConstraints& APConstraints::withJerk(double newJerk) {
  jerk = newJerk;
  return *this;
}
