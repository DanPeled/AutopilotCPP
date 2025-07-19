// Copyright (c) 2025 Dan Peled
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
//
// Inspired by the Autopilot project in Java:
// https://github.com/therekrab/autopilot/

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include <memory>

#include "Constants.h"
#include "subsystems/drivetrain/drivetrain.h"

#define MAKE_UNIQUE_BASE(Base, Derived, ...) \
  std::unique_ptr<Base> {                    \
    std::make_unique<Derived>(__VA_ARGS__)   \
  }

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{
      OperatorConstants::kDriverControllerPort};

  void ConfigureBindings();
  std::optional<TankDrive> m_drive;
};
