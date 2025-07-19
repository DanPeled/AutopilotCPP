// Copyright (c) 2025 Dan Peled
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
//
// Inspired by the Autopilot project in Java:
// https://github.com/therekrab/autopilot/

#include "commands/align.h"

AlignCommand::AlignCommand(TankDrive* drive, const autopilot::APTarget& target,
                           autopilot::Autopilot* autopilot)
    : m_drive{drive}, m_target{target}, m_autopilot{autopilot} {}

void AlignCommand::Execute() {
  frc::Translation2d velocity = m_drive->GetFieldRelativeSpeeds();
  frc::Pose2d pose = m_drive->GetPose();

  frc::Transform2d output = m_autopilot->Calculate(pose, velocity, m_target);

  units::meter_t vX = output.X();
  units::meter_t vY = output.Y();
  frc::Rotation2d headingReference = output.Rotation();

  m_drive->SetFieldRelativeSpeeds(vX, vY, headingReference.Radians());
}

bool AlignCommand::IsFinished() {
  return m_autopilot->AtTarget(m_drive->GetPose(), m_target);
}

void AlignCommand::End(bool inturrupted) {
  m_drive->Stop();
}

AlignCommand& AlignCommand::WithTarget(autopilot::APTarget target) {
  this->m_target = target;
  return *this;
}
