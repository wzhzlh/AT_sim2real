/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GAIT_CLOCK_HPP
#define GAIT_CLOCK_HPP

#include <vector>

#include <yaml-cpp/yaml.h>

// Parameters of the fixed-frequency trot clock. The field names match the
// "gait_clock" block exported by robot_lab (robot_lab/gait.py:TrotClockCfg), so
// a config generated from policy_metadata.json can be used without renaming.
struct GaitClockCfg
{
    float cycle_time = 0.5f;
    float duty_factor = 0.6f;
    float transition_width = 0.05f;
    float linear_enter = 0.08f;
    float linear_exit = 0.04f;
    float angular_enter = 0.10f;
    float angular_exit = 0.05f;

    void Validate() const;
};

/**
 * @brief Command-driven gait clock, ported from robot_lab/gait.py:TrotClock.
 *
 * The phase advances only while the velocity command asks for motion. Both
 * translation and yaw use enter/exit hysteresis so that a command hovering at
 * the threshold does not chatter. When the command stops, the clock keeps
 * advancing until the next planned double support and then freezes there; the
 * phase is never reset to zero mid-episode, so the policy always sees a clock
 * consistent with the gait it has been walking.
 *
 * Update once per control step, after the observation of that step has been
 * consumed, so the next observation receives the advanced phase.
 */
class GaitClock
{
public:
    /**
     * @brief Configure from the "gait_clock" YAML node.
     * @param node The gait_clock node; may be undefined.
     * @param control_dt Control period in seconds (dt * decimation).
     * @return False when the node is missing, i.e. the task does not use the clock.
     */
    bool ConfigureFromYaml(const YAML::Node& node, float control_dt);

    bool Enabled() const { return enabled_; }
    const GaitClockCfg& Config() const { return cfg_; }
    float Phase() const { return phase_; }
    bool Active() const { return moving_ || stopping_; }

    void Reset();
    /** @brief Overwrite the phase (wrapped into [0, 1)); motion flags are kept. */
    void SetPhase(float phase);
    void Update(const std::vector<float>& command, float dt);
    std::vector<float> Observation() const;

private:
    bool enabled_ = false;
    GaitClockCfg cfg_;

    float phase_ = 0.0f;
    bool translating_ = false;
    bool turning_ = false;
    bool moving_ = false;
    bool stopping_ = false;
};

#endif // GAIT_CLOCK_HPP
