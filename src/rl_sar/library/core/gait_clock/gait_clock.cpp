/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gait_clock.hpp"
#include "logger.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kPhaseEpsilon = 1.0e-6f;
constexpr float kControlDtTolerance = 1.0e-3f;

float RequiredAsFloat(const YAML::Node& node, const char *name)
{
    const YAML::Node value = node[name];
    if (!value)
    {
        throw std::runtime_error(std::string("gait_clock is missing required field '") + name + "'");
    }
    return value.as<float>();
}
} // namespace

void GaitClockCfg::Validate() const
{
    if (!std::isfinite(cycle_time) || cycle_time <= 0.0f)
    {
        throw std::runtime_error("gait_clock.cycle_time must be finite and positive");
    }
    if (!(duty_factor > 0.5f && duty_factor < 1.0f))
    {
        throw std::runtime_error("gait_clock.duty_factor must satisfy 0.5 < duty_factor < 1.0");
    }
    if (!(transition_width > 0.0f && transition_width < (1.0f - duty_factor) / 2.0f))
    {
        throw std::runtime_error(
            "gait_clock.transition_width must be positive and leave a nonempty swing core");
    }
    if (!(linear_exit >= 0.0f && linear_exit < linear_enter) || !std::isfinite(linear_enter))
    {
        throw std::runtime_error("gait_clock thresholds must satisfy 0 <= exit < enter < infinity");
    }
    if (!(angular_exit >= 0.0f && angular_exit < angular_enter) || !std::isfinite(angular_enter))
    {
        throw std::runtime_error("gait_clock thresholds must satisfy 0 <= exit < enter < infinity");
    }
}

bool GaitClock::ConfigureFromYaml(const YAML::Node& node, float control_dt)
{
    if (!node || !node.IsMap())
    {
        enabled_ = false;
        return false;
    }

    const YAML::Node version = node["version"];
    if (!version || version.as<int>() != 1)
    {
        throw std::runtime_error(
            "Unsupported gait_clock metadata version; expected version: 1 (see robot_lab/gait.py:TrotClock)");
    }

    cfg_.cycle_time = RequiredAsFloat(node, "cycle_time");
    cfg_.duty_factor = RequiredAsFloat(node, "duty_factor");
    cfg_.transition_width = RequiredAsFloat(node, "transition_width");
    cfg_.linear_enter = RequiredAsFloat(node, "linear_enter");
    cfg_.linear_exit = RequiredAsFloat(node, "linear_exit");
    cfg_.angular_enter = RequiredAsFloat(node, "angular_enter");
    cfg_.angular_exit = RequiredAsFloat(node, "angular_exit");
    cfg_.Validate();

    // A mismatched control period silently rescales the gait frequency, so make
    // it visible instead of letting the robot walk at the wrong cadence.
    const YAML::Node exported_dt = node["control_dt"];
    if (exported_dt && std::fabs(exported_dt.as<float>() - control_dt) > kControlDtTolerance)
    {
        std::cout << LOGGER::WARNING << "gait_clock.control_dt=" << exported_dt.as<float>()
                  << " differs from dt*decimation=" << control_dt
                  << "; the exported clock was trained at a different control rate." << std::endl;
    }

    const YAML::Node initial_phase = node["initial_phase"];
    phase_ = initial_phase ? initial_phase.as<float>() : 0.0f;
    translating_ = false;
    turning_ = false;
    moving_ = false;
    stopping_ = false;
    enabled_ = true;
    return true;
}

void GaitClock::Reset()
{
    phase_ = 0.0f;
    translating_ = false;
    turning_ = false;
    moving_ = false;
    stopping_ = false;
}

void GaitClock::SetPhase(float phase)
{
    if (!std::isfinite(phase))
    {
        throw std::runtime_error("gait clock phase must be finite");
    }
    phase_ = std::fmod(phase, 1.0f);
    if (phase_ < 0.0f)
    {
        phase_ += 1.0f;
    }
}

void GaitClock::Update(const std::vector<float>& command, float dt)
{
    if (!enabled_)
    {
        return;
    }
    if (!std::isfinite(dt) || dt <= 0.0f)
    {
        throw std::runtime_error("gait clock dt must be finite and positive");
    }

    const float linear = command.size() >= 2
        ? std::sqrt(command[0] * command[0] + command[1] * command[1])
        : 0.0f;
    const float angular = command.size() >= 3 ? std::fabs(command[2]) : 0.0f;

    translating_ = translating_ ? (linear > cfg_.linear_exit) : (linear >= cfg_.linear_enter);
    turning_ = turning_ ? (angular > cfg_.angular_exit) : (angular >= cfg_.angular_enter);
    moving_ = translating_ || turning_;

    // A phase resting inside double support must stay still instead of starting
    // an unrequested swing, so a stopped clock only brakes outside that window.
    const float half_phase = std::fmod(phase_, 0.5f);
    const bool in_support = half_phase <= (cfg_.duty_factor - 0.5f) + kPhaseEpsilon;
    stopping_ = !moving_ && !in_support;

    const float until_landing = 0.5f - half_phase;
    float advance = dt / cfg_.cycle_time;
    if (stopping_)
    {
        advance = std::min(advance, until_landing);
    }

    if (moving_ || stopping_)
    {
        phase_ = std::fmod(phase_ + advance, 1.0f);
    }
    if (stopping_ && advance >= until_landing - kPhaseEpsilon)
    {
        stopping_ = false;
    }
}

std::vector<float> GaitClock::Observation() const
{
    const float angle = 2.0f * kPi * phase_;
    return {std::sin(angle), std::cos(angle)};
}
