#include "Hammer_Down.h"
#include <mc_rtc/logging.h>

#include "../HammeringTaskNew.h"

void Hammer_Down::configure(const mc_rtc::Configuration & config)
{
    _config.load(config);
}

void Hammer_Down::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
    load_params();

    _end_point = ctl.robots().robot(ctl.nail_robot_name).frame(ctl.nail_frame_name).position().translation() + Eigen::Vector3d(0, 0, -0.3);
    _target = sva::PTransformd(sva::RotX(M_PI)) * sva::PTransformd(sva::RotY(M_PI/2)) * sva::PTransformd(ctl.robots().robot(ctl.nail_robot_name).frame(ctl.nail_frame_name).position().rotation()) *sva::PTransformd(_end_point);//* sva::PTransformd(Eigen::Vector3d(0.5, 0.2, 1));

    auto gripper_target = sva::PTransformd(Eigen::Quaterniond(0.0f, 0.708, 0.0f, -0.705)) * sva::PTransformd(_end_point);//sva::PTransformd(Eigen::Vector3d(0.7, 0.5, 1)) *

    gripper_task = std::make_shared<mc_tasks::TransformTask>(ctl.robot().frame(ctl.hammer_head_frame_name), _gripper_task_stiffness, _gripper_task_weight);
    ctl.solver().addTask(gripper_task);
    gripper_task->target(gripper_target);

    Eigen::Vector6d dimweights_grip = gripper_task->dimWeight();
    // Remove the orientation part of the BSpline by setting the orientation weights to 0
    dimweights_grip(0) = 0;
    dimweights_grip(1) = 0;
    dimweights_grip(2) = 0;
    // Increase the weights on the x and y coordinates
    dimweights_grip(3) = _gripper_task_dimweight_x;
    dimweights_grip(4) = _gripper_task_dimweight_y;
    dimweights_grip(5) = _gripper_task_dimweight_z;
    gripper_task->dimWeight(dimweights_grip);

    _vectorOrientationTask = std::make_shared<mc_tasks::VectorOrientationTask>(ctl.robot().frame(ctl.hammer_head_frame_name),
                                                                              ctl.normal_vector_to_align_in_hammerhead_frame
    );
    _vectorOrientationTask->targetVector(-ctl.nail_normal_vector_world_frame);
    _vectorOrientationTask->weight(_vector_orientation_task_weight);
    _vectorOrientationTask->stiffness(_vector_orientation_task_stiffness);
    ctl.solver().addTask(_vectorOrientationTask);

}

bool Hammer_Down::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<HammeringTaskNew &>(ctl_);

    ctl.hammer_tip_reference_position_vector = gripper_task->target().translation();
    ctl.hammer_tip_actual_velocity_vector = ctl.robot().frame(ctl.hammer_head_frame_name).velocity().linear();
    ctl.hammer_tip_actual_position_vector = ctl.robot().frame(ctl.hammer_head_frame_name).position().translation();

    ctl.impact_detected = abs(ctl.nail_force_vector.x()) >= ctl.magic_force_threshold ||
                          abs(ctl.nail_force_vector.y()) >= ctl.magic_force_threshold ||
                          abs(ctl.nail_force_vector.z()) >= ctl.magic_force_threshold;


    stop = (gripper_task->eval()).norm() < 1e-3;

    if(ctl.impact_detected)
    {
        Eigen::Vector3d hammer_normal_world_frame = (ctl.robot().frame(ctl.hammer_head_frame_name).position().rotation().transpose()*Eigen::Vector3d(1, 0, 0)).normalized();

        mc_rtc::log::info("IMPACT DETECTED ON THE NAIL");

        mc_rtc::log::info("actual hammer normal in world frame = {}", hammer_normal_world_frame);
        mc_rtc::log::info("target hammer normal in world frame = {}", -ctl.nail_normal_vector_world_frame);
        mc_rtc::log::info("angle error = {} deg", (180/M_PI) * _vectorOrientationTask->eval().norm());// vector_error(hammer_normal_world_frame, -ctl.nail_normal_vector_world_frame));

        output("STOP");
        return true;
    }
    return false;
}

void Hammer_Down::teardown(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
    ctl.solver().removeTask(gripper_task);
    ctl.solver().removeTask(_vectorOrientationTask);

}

void Hammer_Down::load_params()
{
    std::string magic_values_key = "magic_values";

    _gripper_task_weight = _config(magic_values_key)("gripper_task_weight");
    _gripper_task_stiffness = _config(magic_values_key)("gripper_task_min_stiffness");
    _gripper_task_dimweight_x = _config(magic_values_key)("magic_BSpline_task_dimweight_x");
    _gripper_task_dimweight_y = _config(magic_values_key)("magic_BSpline_task_dimweight_y");
    _gripper_task_dimweight_z = _config(magic_values_key)("magic_BSpline_task_dimweight_z");

    _vector_orientation_task_weight = _config(magic_values_key)("magic_vector_orientation_task_weight");
    _vector_orientation_task_stiffness = _config(magic_values_key)("magic_vector_orientation_task_stiffness");

}

EXPORT_SINGLE_STATE("Hammer_Down", Hammer_Down)
