#pragma once

#include <mc_control/fsm/State.h>
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/PostureTask.h>
#include <memory>

struct Post_Impact_Task : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl_) override;

  bool run(mc_control::fsm::Controller & ctl_) override;

  void teardown(mc_control::fsm::Controller & ctl_) override;

  private:

    std::shared_ptr<mc_tasks::PostureTask> _postureTask;
    /**
    @brief Returns the values needed to modify the posture task to the half sitting posture (the default posture)
     */
    const std::vector<std::vector<double>> getHalfSittingPositionVector() const;

    // ---------------- Parameters -----------------
    mc_rtc::Configuration _config;
    double _magic_posture_task_weight = 1.0f;
    double _magic_posture_task_stiffness = 1.0f;
    double _magic_posture_task_epsilon = 0.1f;


    void load_parameters();

};
