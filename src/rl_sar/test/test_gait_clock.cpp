/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

// Numeric regressions for the deployed gait clock. They mirror
// tests/test_gait.py on the robot_lab side, so the two implementations stay in
// sync without needing PyTorch here.
//
// Build and run manually (from src/rl_sar):
//   g++ -std=c++17 -I library/core/gait_clock -I library/core/logger
//       test/test_gait_clock.cpp library/core/gait_clock/gait_clock.cpp
//       -lyaml-cpp -o /tmp/test_gait_clock && /tmp/test_gait_clock

#include "gait_clock.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
int g_failures = 0;

void Expect(bool condition, const std::string& what)
{
    if (!condition)
    {
        std::cerr << "FAILED: " << what << std::endl;
        ++g_failures;
    }
}

void ExpectNear(float actual, float expected, float tolerance, const std::string& what)
{
    Expect(std::fabs(actual - expected) <= tolerance,
           what + " (actual=" + std::to_string(actual) + ", expected=" + std::to_string(expected) + ")");
}

const char *kDefaultMetadata = R"(
version: 1
cycle_time: 0.5
duty_factor: 0.6
transition_width: 0.05
linear_enter: 0.08
linear_exit: 0.04
angular_enter: 0.10
angular_exit: 0.05
control_dt: 0.02
initial_phase: 0.0
)";

GaitClock MakeClock()
{
    GaitClock clock;
    clock.ConfigureFromYaml(YAML::Load(kDefaultMetadata), 0.02f);
    return clock;
}

float Wrap(float value)
{
    float wrapped = std::fmod(value, 1.0f);
    return wrapped < 0.0f ? wrapped + 1.0f : wrapped;
}
} // namespace

int main()
{
    // Fixed frequency: 25 steps of 0.02 s cover one 0.5 s cycle exactly.
    {
        GaitClock clock = MakeClock();
        const std::vector<float> moving = {0.5f, 0.0f, 0.0f};
        const std::vector<float> turning = {0.0f, 0.0f, 0.4f};
        for (int i = 0; i < 25; ++i)
        {
            clock.Update(i % 2 == 0 ? moving : turning, 0.02f);
        }
        Expect(clock.Active(), "clock stays active while commanded");
        ExpectNear(clock.Phase(), 0.0f, 1.0e-5f, "one cycle returns to phase zero");
        const std::vector<float> observation = clock.Observation();
        ExpectNear(observation[0], 0.0f, 1.0e-4f, "phase encoding sin");
        ExpectNear(observation[1], 1.0f, 1.0e-6f, "phase encoding cos");

        // Reading the observation must not advance the clock.
        const float before = clock.Phase();
        clock.Observation();
        ExpectNear(clock.Phase(), before, 0.0f, "observation is side-effect free");
    }

    // Independent linear/angular hysteresis.
    {
        GaitClock clock = MakeClock();
        struct Case
        {
            float speed;
            float yaw;
            bool expected;
        };
        const Case cases[] = {{0.06f, 0.08f, false}, {0.09f, 0.11f, true},
                              {0.06f, 0.08f, true}, {0.03f, 0.04f, false}};
        for (const Case& item : cases)
        {
            clock.Update({item.speed, 0.0f, item.yaw}, 0.02f);
            Expect(clock.Active() == item.expected,
                   "hysteresis state for speed=" + std::to_string(item.speed));
        }
    }

    // Stopping always lands inside the planned double support, without a jump.
    {
        for (int i = 0; i < 1000; ++i)
        {
            GaitClock clock = MakeClock();
            clock.SetPhase(static_cast<float>(i) / 1000.0f);
            const std::vector<float> zero = {0.0f, 0.0f, 0.0f};
            for (int step = 0; step < 14; ++step)
            {
                const float previous = clock.Phase();
                clock.Update(zero, 0.02f);
                const float delta = Wrap(clock.Phase() - previous);
                Expect(delta <= 0.040001f, "stop advances at most one control step of phase");
            }
            Expect(!clock.Active(), "stopped clock is inactive");
            const float half_phase = std::fmod(clock.Phase(), 0.5f);
            Expect(half_phase <= 0.1f + 1.0e-6f, "frozen clock sits in double support");

            const float frozen = clock.Phase();
            clock.Update(zero, 0.02f);
            ExpectNear(clock.Phase(), frozen, 0.0f, "frozen clock stays frozen");

            clock.Update({0.5f, 0.0f, 0.0f}, 0.02f);
            ExpectNear(Wrap(clock.Phase() - frozen), 0.04f, 1.0e-5f,
                       "restart continues from the landing phase without a jump");
        }
    }

    // Reset semantics and metadata validation.
    {
        GaitClock clock = MakeClock();
        clock.Update({0.5f, 0.0f, 0.0f}, 0.02f);
        clock.Reset();
        ExpectNear(clock.Phase(), 0.0f, 0.0f, "reset returns to phase zero");
        Expect(!clock.Active(), "reset clears motion state");

        bool threw = false;
        try
        {
            GaitClock invalid;
            invalid.ConfigureFromYaml(YAML::Load("version: 1\ncycle_time: 0.5\n"), 0.02f);
        }
        catch (const std::runtime_error&)
        {
            threw = true;
        }
        Expect(threw, "missing gait_clock fields are rejected");

        threw = false;
        try
        {
            GaitClock stale;
            stale.ConfigureFromYaml(YAML::Load("{version: 0, cycle_time: 0.5}"), 0.02f);
        }
        catch (const std::runtime_error&)
        {
            threw = true;
        }
        Expect(threw, "unsupported metadata version is rejected");

        GaitClock absent;
        Expect(!absent.ConfigureFromYaml(YAML::Node(), 0.02f), "missing gait_clock block disables the clock");
    }

    if (g_failures == 0)
    {
        std::cout << "test_gait_clock: all checks passed" << std::endl;
    }
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
