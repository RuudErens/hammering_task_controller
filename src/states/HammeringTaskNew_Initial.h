#pragma once

#include <mc_control/fsm/State.h>

struct HammeringTaskNew_Initial : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  private:
    bool _positionning_hammer_clicked = false;
    double total_time_elapsed;
    bool stabilizer_reset_done = false;


    bool first_iteration = false;
    double first_instance_error = 0;
    double _posture_task_max_stiffness = 10;
    double _posture_task_goal_error = 0.01;
    double _posture_task_K_scaling_factor = 0.5;
};
