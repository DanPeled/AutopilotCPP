#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/velocity.h>
#include <optional>

namespace autopilot
{
  /**
   * A class representing the goal end state of an autopilot action
   *
   * A target needs a reference Pose2d, but can optionally have a specified entry angle and rotation
   * radius
   *
   * A target may also specify an end velocity.
   *
   * The target also may have a desired end velocity.
   */
  class APTarget
  {
  protected:
    frc::Pose2d m_reference;
    std::optional<frc::Rotation2d> m_entryAngle;
    units::meters_per_second_t m_velocity;
    std::optional<units::meter_t> m_rotationRadius;

  public:
    /**
     * Creates a new autopilot target with the given target pose, no entry angle, and no end velocity.
     *
     * @param pose The reference pose for this target.
     */
    APTarget(const frc::Pose2d &pose);

    /**
     * Returns a copy of this target with the given reference.
     *
     * @param reference The reference pose for this target.
     */
    [[nodiscard]] APTarget WithReference(const frc::Pose2d &reference) const;

    /**
     * Returns a copy of this target with the given entry angle.
     *
     * @param entryAngle The entry angle for the new target.
     */
    [[nodiscard]] APTarget WithEntryAngle(const frc::Rotation2d &angle) const;

    /**
     * Returns a copy of this target with the given end velocity. Note that if the robot does not
     * reach this velocity, no issues will be thrown. Autopilot will only try to reach this target,
     * but it will not be affected by whether it does.
     *
     * @param velocity The desired end velocity when the robot approaches the target
     */
    [[nodiscard]] APTarget WithVelocity(units::meters_per_second_t velocity) const;

    /**
     * Returns a copy of this target with the given rotation radius.
     *
     * Rotation radius is the distance from the target pose that rotation goals are respected. By
     * default, rotation goals are always respected, but if autopilot shouldn't reorient the robot
     * until X distance from setpoint, this can be used to make that change.
     *
     * @param radius The rotation radius for the new target
     */
    [[nodiscard]] APTarget WithRotationRadius(units::meter_t radius) const;

    /**
     * Returns this target's reference pose.
     */
    [[nodiscard]] const frc::Pose2d &Reference() const;

    /**
     * Returns this target's optional entry angle.
     */
    [[nodiscard]] const std::optional<frc::Rotation2d> &EntryAngle() const;

    /**
     * Returns this target's rotation radius.
     */
    [[nodiscard]] const std::optional<units::meter_t> &RotationRadius() const;

    /**
     * Returns this target's end velocity.
     */
    [[nodiscard]] units::meters_per_second_t Velocity() const;

    /**
     * Returns a copy of this target.
     */
    [[nodiscard]] APTarget Clone() const
    {
      return *this;
    }

    /**
     * Retuns a copy of this target, without the entry angle set. This is useful if trying to make two
     * different targets with and without entry angle set.
     */
    [[nodiscard]] APTarget WithoutEntryAngle() const;
  };
} // namespace autopilot
