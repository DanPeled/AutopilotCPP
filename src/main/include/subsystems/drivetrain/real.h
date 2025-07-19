// Copyright (c) 2025 Dan Peled
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
//
// Inspired by the Autopilot project in Java:
// https://github.com/therekrab/autopilot/

#pragma once

#include "drivetrain.h"

class TankDriveIOReal : public TankDriveIO {
 public:
  TankDriveIOReal() = default;

  void SetLeft(double output) override;
  void SetRight(double output) override;

  frc::Rotation2d GetRotation() const override;
  units::meter_t GetLeftPos() const override;
  units::meter_t GetRightPos() const override;
};
