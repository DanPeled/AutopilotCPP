// Copyright (c) 2025 Dan Peled
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
//
// Inspired by the Autopilot project in Java:
// https://github.com/therekrab/autopilot/

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "autopilot/autopilot.h"
#include "autopilot/target.h"
#include "subsystems/drivetrain/drivetrain.h"

class AlignCommand : public frc2::CommandHelper<frc2::Command, AlignCommand> {
 public:
  explicit AlignCommand(TankDrive* drive, const autopilot::APTarget& target,
                        autopilot::Autopilot* autopilot);

  AlignCommand& WithTarget(autopilot::APTarget target);

  void Execute() override;
  bool IsFinished() override;
  void End(bool inturrupted) override;

 private:
  autopilot::APTarget m_target;
  autopilot::Autopilot* m_autopilot;
  TankDrive* m_drive;
};
