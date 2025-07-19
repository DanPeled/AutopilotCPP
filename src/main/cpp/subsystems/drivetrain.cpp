// Copyright (c) 2025 Dan Peled
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
//
// Inspired by the Autopilot project in Java:
// https://github.com/therekrab/autopilot/

#include "subsystems/drivetrain/drivetrain.h"
#include "subsystems/drivetrain/real.h"
#include "subsystems/drivetrain/sim.h"

#include <utility>

TankDrive::TankDrive(std::unique_ptr<TankDriveIO> io, units::meter_t trackWidth)
    : m_kinematics{trackWidth},
      m_odometry{io->GetRotation(), io->GetLeftPos(), io->GetRightPos(),
                 frc::Pose2d{}},
      m_io{std::move(io)} {}

frc::Pose2d TankDrive::GetPose() {
  return frc::Pose2d();
}

frc::Translation2d TankDrive::GetFieldRelativeSpeeds() {
  return frc::Translation2d();
}

void TankDrive::SetFieldRelativeSpeeds(units::meter_t vX, units::meter_t vY,
                                       units::radian_t theta) {}
void TankDrive::Stop() {}

TankDriveIOSim::TankDriveIOSim() {}