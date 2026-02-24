#include "HammeringTaskNew_Initial.h"

#include <mc_rtc/clock.h>
#include <mc_rtc/logging.h>


void HammeringTaskNew_Initial::configure(const mc_rtc::Configuration& config)
{
   _config.load(config);
}

void HammeringTaskNew_Initial::start(mc_control::fsm::Controller& ctl_)
{
    auto& ctl = static_cast<HammeringTaskNew&>(ctl_);

    load_params();

    // Creates a button to start the movement
    ctl.gui()->addElement({}, mc_rtc::gui::Button("Start hammering", [this]()
    {
        positioning_hammer_clicked = true;
    }));

    // Set posture task stiffness and weight
    ctl.getPostureTask(ctl.robot().name())->stiffness(ctl.base_posture_stiffness);
    ctl.getPostureTask(ctl.robot().name())->weight(ctl.base_posture_weight);

    mc_rtc::log::info("Starting Initial State");
    total_time_elapsed = 0.0f;
}

bool HammeringTaskNew_Initial::run(mc_control::fsm::Controller& ctl_)
{
    auto& ctl = static_cast<HammeringTaskNew&>(ctl_);
    total_time_elapsed += ctl_.solver().dt();

    // make the simulation automatically stop if the max number of hits is reached
    if (ctl.number_of_hits >= ctl.max_number_of_hits && total_time_elapsed > automatic_transition_time)
    {
        mc_rtc::log::error_and_throw("This simulation is done according to the maximum hit iterations");
    }

    // mc_rtc::log::info("In Initial State, time elapsed: {}s, the transition time is {}, the eval {} is and the goal is {}", total_time_elapsed, automatic_transition_time, ctl.stabilizing_eval_norm, stabilization_eval_norm_goal);

    // Transition to next state based on time or button clicked and other transition conditions (stabilization_eval_norm and number of hits)
    if ((positioning_hammer_clicked || total_time_elapsed > automatic_transition_time) && ctl.stabilizing_eval_norm < stabilization_eval_norm_goal && ctl.
        number_of_hits < ctl.max_number_of_hits)
    {
        output("BUTTON_CLICKED");
        return true;
    }
    return false;
}

void HammeringTaskNew_Initial::teardown(mc_control::fsm::Controller& ctl_)
{
    auto& ctl = static_cast<HammeringTaskNew&>(ctl_);
    ctl_.gui()->removeElement({}, "Start hammering");
}

void HammeringTaskNew_Initial::load_params()
{
    // --------------- Loading state parameters ------------------------
    const std::string magic_values_key = "parameters";
    automatic_transition_time = _config(magic_values_key)("automatic_transition_time");
    stabilization_eval_norm_goal = _config(magic_values_key)("stabilization_eval_norm_goal");
}

EXPORT_SINGLE_STATE("HammeringTaskNew_Initial", HammeringTaskNew_Initial)
