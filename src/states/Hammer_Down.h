#pragma once

#include <mc_control/fsm/State.h>

#include <mc_tasks/TransformTask.h>
#include <mc_tasks/VectorOrientationTask.h>

struct Hammer_Down : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

    void load_params();

private:
    mc_rtc::Configuration _config;

    Eigen::Vector3d _end_point;
    sva::PTransformd _target;
    std::shared_ptr<mc_tasks::TransformTask> gripper_task;
    double _gripper_task_stiffness = 0;
    double _gripper_task_weight = 0;

    double _gripper_task_dimweight_x = 1.0f;
    double _gripper_task_dimweight_y = 1.0f;
    double _gripper_task_dimweight_z = 1.0f;

    std::shared_ptr<mc_tasks::VectorOrientationTask> _vectorOrientationTask;

    double _vector_orientation_task_weight = 1.f;
    double _vector_orientation_task_stiffness = 1.f;

    bool stop = false;
};
