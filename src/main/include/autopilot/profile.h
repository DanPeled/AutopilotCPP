// Copyright (c) 2025 Dan Peled
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
//
// Inspired by the Autopilot project in Java:
// https://github.com/therekrab/autopilot/

#pragma once

#include <units/angle.h>

#include "constraints.h"

namespace autopilot {
/**
 * A class representing a profile that determines how AP approaches a target.
 *
 * The constraints property of the profile limits the robot's behavior.
 *
 * Acceptable error for the controller (both translational and rotational) are
 * stored here.
 *
 * The "beeline radius" determines the distance at which the robot drives
 * directly at the target and no longer respects entry angle. This is helpful
 * because if the robot overshoots by a small amount, that error should not
 * cause the robot do completely circle back around.
 */
class APProfile {
 protected:
  APConstraints m_constraints;
  units::meter_t m_errorXY;
  units::radian_t m_errorTheta;
  units::meter_t m_beelineRadius;

 public:
  /**
   * Builds an APProfile with the given constraints. Tolerated error and beeline
   * radius are all set to zero.
   *
   * @param constraints The motion constraints for this profile
   */
  explicit APProfile(const APConstraints& constraints);

  /**
   * Modifies this profile's tolerated error in the XY plane and returns itself
   */
  APProfile& WithErrorXY(units::meter_t errorXY);

  /**
   * Modifies this profile's tolerated angular error and returns itself
   */
  APProfile& WithErrorTheta(units::radian_t errorTheta);

  /**
   * Modifies this profile's path generation constraints and returns itself
   */
  APProfile& WithConstraints(const APConstraints& constraints);

  /**
   * Modifies this profile's beeline radius and returns itself
   *
   * The beeline radius is a distance where, under that range, entry angle is no
   * longer respected. This prevents small overshoots from causing the robot to
   * make a full arc and instaed correct itself.
   */
  APProfile& WithBeelineRadius(units::meter_t beelineRadius);

  /**
   * Returns the tolerated translation error for this profile
   */
  units::meter_t ErrorXY() const;

  /**
   * Returns the tolerated angular error for this profile
   */
  units::radian_t ErrorTheta() const;

  /**
   * Returns the path generation constraints for this profile
   */
  const APConstraints& Constraints() const;

  /**
   * Returns the beeline radius for this profile
   */
  units::meter_t BeelineRadius() const;
};
}  // namespace autopilot
