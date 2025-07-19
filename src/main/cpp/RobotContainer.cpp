// Copyright (c) 2025 Dan Peled
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
//
// Inspired by the Autopilot project in Java:
// https://github.com/therekrab/autopilot/

#include "RobotContainer.h"

#include <frc/RobotBase.h>
#include <frc2/command/button/Trigger.h>

#include <memory>

#include "subsystems/drivetrain/real.h"
#include "subsystems/drivetrain/sim.h"

RobotContainer::RobotContainer() {
  std::unique_ptr<TankDriveIO> driveIO =
      frc::RobotBase::IsSimulation()
          ? MAKE_UNIQUE_BASE(TankDriveIO, TankDriveIOSim)
          : MAKE_UNIQUE_BASE(TankDriveIO, TankDriveIOReal);

  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::CommandPtr{std::unique_ptr<frc2::Command>{}};
}
