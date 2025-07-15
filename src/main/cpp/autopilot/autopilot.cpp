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

    units::meter_t initialI = adjustedInitial.X();
    units::meter_t goalI = adjustedGoal.X();

    if (goalI.value() > m_profile.Constraints().velocity.value())
    {
        goalI = units::meter_t{m_profile.Constraints().velocity.value()};
    }

    double adjustedI = std::min(goalI.value(), Push(initialI.value(), goalI.value(), m_profile.Constraints().acceleration));
    return frc::Translation2d(units::meter_t{adjustedI}, units::meter_t{0}).RotateBy(angleOffset);
}

double Autopilot::Push(double start, double end, units::meters_per_second_squared_t accel)
{
    units::meters_per_second_t maxChange = accel * dt;
    if (std::abs(start - end) < maxChange.value())
    {
        return end;
    }
    return start > end ? start - maxChange.value() : start + maxChange.value();
}

frc::Translation2d Autopilot::CalculateSwirlyVelocity(const frc::Translation2d &offset, const APTarget &target)
{
    units::meter_t disp = offset.Norm();                           // displacement magnitude (meter_t)
    frc::Rotation2d theta(offset.X().value(), offset.Y().value()); // construct Rotation2d from raw doubles
    units::radian_t rads = theta.Radians();                        // angle in radians
    units::meter_t dist = CalculateSwirlyLength(rads, disp);

    // Calculate vx, vy as unitless doubles (direction vector components)
    double vx = theta.Cos() - rads.value() * theta.Sin();
    double vy = rads.value() * theta.Cos() + theta.Sin();

    frc::Translation2d translation{units::meter_t{vx}, units::meter_t{vy}}; // Translation2d takes doubles (unitless)

    units::meter_t norm = translation.Norm(); // norm is a meter_t because Translation2d uses units::meter_t for components

    if (norm.value() == 0.0)
    {
        return frc::Translation2d(); // zero vector if no direction
    }

    // Normalize translation vector (unitless), so divide components by norm (units::meter_t)
    frc::Translation2d unitVec = frc::Translation2d(translation.X() / norm.value(), translation.Y() / norm.value());

    // Calculate speed (units::meter_t), extract raw double with .value()
    double speed = CalculateMaxVelocity(dist, target.Velocity()).value();

    // Multiply normalized direction (unitless) by scalar speed (double) to get velocity vector
    return unitVec * speed;
}

units::meter_t Autopilot::CalculateSwirlyLength(units::radian_t theta, units::meter_t radius)
{
    if (theta.value() == 0.0)
        return radius;

    const double thetaVal = std::abs(theta.value());
    const double hypot = std::hypot(thetaVal, 1.0);
    const double logTerm = std::log(thetaVal + hypot);
    units::meter_t u1 = radius * hypot;
    units::meter_t u2 = radius * (logTerm / thetaVal);
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
