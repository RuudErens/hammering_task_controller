#pragma once

#include <mc_control/fsm/State.h>
#include "../HammeringTaskNew.h"

struct HammeringTaskNew_Initial : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration& config) override;

    void start(mc_control::fsm::Controller& ctl) override;

    bool run(mc_control::fsm::Controller& ctl) override;

    void teardown(mc_control::fsm::Controller& ctl) override;

private:
    /**
    @brief Loads the parameters found in the configuration yaml-file (inputs in etc/HammeringTaskNew.in.yaml)
    */
    void load_params();

    mc_rtc::Configuration _config;

    bool positioning_hammer_clicked = false;
    double total_time_elapsed = 0.0f;

    double automatic_transition_time = 0.0f;
    double stabilization_eval_norm_goal = 0.0f;
};
