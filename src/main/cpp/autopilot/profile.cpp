// Copyright (c) 2025 Dan Peled
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
//
// Inspired by the Autopilot project in Java:
// https://github.com/therekrab/autopilot/

#include "autopilot/profile.h"

using namespace autopilot;

APProfile::APProfile(const APConstraints& constraints)
    : m_constraints{constraints},
      m_errorXY{0},
      m_errorTheta{0},
      m_beelineRadius{0} {}

APProfile& APProfile::WithErrorXY(units::meter_t errorXY) {
  this->m_errorXY = errorXY;
  return *this;
}

APProfile& APProfile::WithErrorTheta(units::radian_t errorTheta) {
  this->m_errorTheta = errorTheta;
  return *this;
}

APProfile& APProfile::WithConstraints(const APConstraints& constraints) {
  this->m_constraints = constraints;
  return *this;
}

APProfile& APProfile::WithBeelineRadius(units::meter_t beelineRadius) {
  this->m_beelineRadius = beelineRadius;
  return *this;
}

units::meter_t APProfile::ErrorXY() const {
  return m_errorXY;
}

units::radian_t APProfile::ErrorTheta() const {
  return m_errorTheta;
}

const APConstraints& APProfile::Constraints() const {
  return m_constraints;
}

units::meter_t APProfile::BeelineRadius() const {
  return m_beelineRadius;
}
