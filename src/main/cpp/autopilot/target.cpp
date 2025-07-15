#include "autopilot/target.h"

using namespace autopilot;

APTarget::APTarget(const frc::Pose2d &pose) : m_reference(pose), m_velocity{0_mps}, m_entryAngle{}, m_rotationRadius{}
{
}

APTarget APTarget::WithReference(const frc::Pose2d &reference) const
{
    APTarget target = this->Clone();
    target.m_reference = reference;
    return target;
}

APTarget APTarget::WithEntryAngle(const frc::Rotation2d &entryAngle) const
{
    APTarget target = this->Clone();
    target.m_entryAngle = std::optional<frc::Rotation2d>{entryAngle};
    return target;
}

APTarget APTarget::WithVelocity(units::meters_per_second_t velocity) const
{
    APTarget target = this->Clone();
    target.m_velocity = velocity;
    return target;
}

APTarget APTarget::WithRotationRadius(units::meter_t radius) const
{
    APTarget target = this->Clone();
    target.m_rotationRadius = std::optional<units::meter_t>{radius};
    return target;
}

const frc::Pose2d &APTarget::Reference() const
{
    return this->m_reference;
}

const std::optional<frc::Rotation2d> &APTarget::EntryAngle() const
{
    return this->m_entryAngle;
}

units::meters_per_second_t APTarget::Velocity() const
{
    return this->m_velocity;
}

APTarget autopilot::APTarget::WithoutEntryAngle() const
{
    APTarget target{this->m_reference};
    target.m_velocity = this->m_velocity;
    target.m_rotationRadius = this->m_rotationRadius;
    return target;
}

const std::optional<units::meter_t> &APTarget::RotationRadius() const
{
    return this->m_rotationRadius;
}
