// Copyright (c) 2025 Dan Peled
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
//
// Inspired by the Autopilot project in Java:
// https://github.com/therekrab/autopilot/

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc2/command/SubsystemBase.h>

#include <memory>

#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/kinematics/DifferentialDriveOdometry.h"

class TankDriveIO {
 public:
  virtual ~TankDriveIO() {}

  // Inputs
  virtual void SetLeft(double output) = 0;
  virtual void SetRight(double output) = 0;

  // Outputs
  virtual frc::Rotation2d GetRotation() const = 0;
  virtual units::meter_t GetLeftPos() const = 0;
  virtual units::meter_t GetRightPos() const = 0;
};

class TankDrive : frc2::SubsystemBase {
 public:
  explicit TankDrive(std::unique_ptr<TankDriveIO> io,
                     units::meter_t trackWidth);

  frc::Pose2d GetPose();
  frc::Translation2d GetFieldRelativeSpeeds();
  void SetFieldRelativeSpeeds(units::meter_t vX, units::meter_t vY,
                              units::radian_t theta);
  void Stop();

 private:
  std::unique_ptr<TankDriveIO> m_io;
  frc::DifferentialDriveKinematics m_kinematics;
  frc::DifferentialDriveOdometry m_odometry;
};
