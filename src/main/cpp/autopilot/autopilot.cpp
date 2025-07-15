#include "autopilot/autopilot.h"

using namespace autopilot;

Autopilot::Autopilot(const APProfile &profile)
    : m_profile(profile) {}

frc::Transform2d Autopilot::Calculate(const frc::Pose2d &current,
                                      const frc::Translation2d &velocity,
                                      const APTarget &target)
{
    frc::Translation2d offset = ToTargetCoordinateFrame(target.Reference().Translation() - current.Translation(), target);

    if (offset == frc::Translation2d())
    {
        return frc::Transform2d(frc::Translation2d(), target.Reference().Rotation());
    }

    frc::Translation2d initial = ToTargetCoordinateFrame(velocity, target);
    units::meter_t disp = offset.Norm();

    if (!target.EntryAngle().has_value() || disp < m_profile.BeelineRadius())
    {
        frc::Translation2d towardsTarget = offset / disp.value();
        frc::Translation2d goal = towardsTarget * CalculateMaxVelocity(disp, target.Velocity()).value();
        frc::Translation2d out = Correct(initial, goal);
        frc::Translation2d velo = ToGlobalCoordinateFrame(out, target);
        frc::Rotation2d rot = GetRotationTarget(current.Rotation(), target, disp);
        return frc::Transform2d(velo, rot);
    }

    frc::Translation2d goal = CalculateSwirlyVelocity(offset, target);
    frc::Translation2d out = Correct(initial, goal);
    frc::Translation2d velo = ToGlobalCoordinateFrame(out, target);
    frc::Rotation2d rot = GetRotationTarget(current.Rotation(), target, disp);
    return frc::Transform2d(velo, rot);
}

frc::Translation2d Autopilot::ToTargetCoordinateFrame(const frc::Translation2d &coords, const APTarget &target)
{
    frc::Rotation2d entryAngle = target.EntryAngle().value_or(frc::Rotation2d());
    return coords.RotateBy(-entryAngle);
}

frc::Translation2d Autopilot::ToGlobalCoordinateFrame(const frc::Translation2d &coords, const APTarget &target)
{
    frc::Rotation2d entryAngle = target.EntryAngle().value_or(frc::Rotation2d());
    return coords.RotateBy(entryAngle);
}

units::meters_per_second_t Autopilot::CalculateMaxVelocity(units::meter_t dist, units::meters_per_second_t endVelo)
{
    return units::meters_per_second_t{std::cbrt((4.5 * std::pow(dist.value(), 2.0)) * m_profile.Constraints().jerk)} + endVelo;
}

frc::Translation2d Autopilot::Correct(const frc::Translation2d &initial, const frc::Translation2d &goal)
{
    frc::Rotation2d angleOffset;
    if (goal != frc::Translation2d())
    {
        angleOffset = frc::Rotation2d(goal.X().value(), goal.Y().value());
    }

    frc::Translation2d adjustedGoal = goal.RotateBy(-angleOffset);
    frc::Translation2d adjustedInitial = initial.RotateBy(-angleOffset);

    units::radian_t initialI = adjustedInitial.X();
    units::radian_t goalI = adjustedGoal.X();

    if (goalI > m_profile.Constraints().velocity)
    {
        goalI = m_profile.Constraints().velocity;
    }

    double adjustedI = std::min(goalI.value(), Push(initialI.value(), goalI.value(), m_profile.Constraints().acceleration));
    return frc::Translation2d(units::meter_t{adjustedI}, units::meter_t{0}).RotateBy(angleOffset);
}

double Autopilot::Push(double start, double end, units::meters_per_second_squared_t accel)
{
    units::meter_t maxChange = accel * dt;
    if (std::abs(start - end) < maxChange.value())
    {
        return end;
    }
    return start > end ? start - maxChange.value() : start + maxChange.value();
}

frc::Translation2d Autopilot::CalculateSwirlyVelocity(const frc::Translation2d &offset, const APTarget &target)
{
    units::meter_t disp = offset.Norm();
    frc::Rotation2d theta(offset.X().value(), offset.Y().value());
    units::radian_t rads = theta.Radians();
    units::meter_t dist = CalculateSwirlyLength(rads, disp);

    units::meter_t vx = units::meter_t{theta.Cos() - rads.value() * theta.Sin()};
    units::meter_t vy = units::meter_t{rads.value() * theta.Cos() + theta.Sin()};

    frc::Translation2d translation{vx, vy};
    units::meter_t norm = translation.Norm();

    if (norm.value() == 0.0)
    {
        return frc::Translation2d(); // Return zero vector if norm is 0 to avoid division by zero
    }

    frc::Translation2d unitVec = frc::Translation2d(translation.X() / norm, translation.Y() / norm);
    return unitVec * CalculateMaxVelocity(dist, target.Velocity()).value();
}

units::meter_t Autopilot::CalculateSwirlyLength(units::radian_t theta, units::meter_t radius)
{
    if (theta.value() == 0.0)
        return radius;

    theta = units::radian_t{std::abs(theta.value())};
    double hypot = std::hypot(theta.value(), 1.0);
    units::meter_t u1 = radius * hypot;
    units::meter_t u2 = radius * std::log(theta + hypot) / theta;
    return units::meter_t{0.5 * (u1 + u2)};
}

frc::Rotation2d Autopilot::GetRotationTarget(const frc::Rotation2d &current, const APTarget &target, units::meter_t dist)
{
    if (!target.RotationRadius().has_value())
    {
        return target.Reference().Rotation();
    }

    units::meter_t radius = target.RotationRadius().value_or(units::meter_t{0});
    if (radius > dist)
    {
        return target.Reference().Rotation();
    }
    return current;
}

bool Autopilot::AtTarget(const frc::Pose2d &current, const APTarget &target)
{
    const frc::Pose2d &goal = target.Reference();
    bool okXY = std::hypot(current.X().value() - goal.X().value(), current.Y().value() - goal.Y().value()) <= m_profile.ErrorXY().to<double>();
    bool okTheta = std::abs((current.Rotation() - goal.Rotation()).Radians().value()) <= m_profile.ErrorTheta().to<double>();
    return okXY && okTheta;
}
