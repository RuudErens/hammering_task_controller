#include "Post_Impact_Task.h"
#include "../HammeringTaskNew.h"
#include <mc_rtc/logging.h>
#include <mc_tasks/PostureTask.h>
#include <memory>


void Post_Impact_Task::configure(const mc_rtc::Configuration & config)
{
    _config.load(config);
    mc_rtc::log::info("Post_Impact_Task configure function called with config :  \n{}", config.dump(true, true));
    load_parameters();
}

void Post_Impact_Task::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<HammeringTaskNew &>(ctl_);

    _postureTask = std::make_shared<mc_tasks::PostureTask>(ctl.solver(), 
                                                            ctl.robot().robotIndex());
    _postureTask->posture(ctl.base_posture_vector);
    _postureTask->stiffness(_magic_posture_task_stiffness);
    _postureTask->weight(_magic_posture_task_weight);
    ctl.getPostureTask(ctl.robot().name())->stiffness(_magic_posture_task_stiffness);
    ctl.getPostureTask(ctl.robot().name())->weight(_magic_posture_task_weight);

    // ctl.solver().addTask(_postureTask);
}

bool Post_Impact_Task::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<HammeringTaskNew &>(ctl_);

    // Find a better condition than that
    if(_postureTask->speed().norm() < 0.03){
        output("STOP");
        return true;
    }
    // return _postureTask->eval().norm() < _magic_posture_task_epsilon && _postureTask->speed().norm() < 0.0003;
    return false;
}

void Post_Impact_Task::teardown(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
    // ctl.solver().removeTask(_postureTask);
}

void Post_Impact_Task::load_parameters()
{
    // --------------- Loading magic values ------------------------
    const std::string magic_values_key = "magic_values";
    _magic_posture_task_weight = _config(magic_values_key)("magic_posture_task_weight");
    _magic_posture_task_stiffness = _config(magic_values_key)("magic_posture_task_stiffness");
    _magic_posture_task_epsilon = _config(magic_values_key)("magic_posture_task_epsilon");

}

const std::vector<std::vector<double>> Post_Impact_Task::getHalfSittingPositionVector() const
{
    // Found in mc_hrp5p
    const std::map<std::string, double> halfSitting
    {
    {"RCY", 0.0},
    {"RCR", 0.0},
    {"RCP", -26.87},
    {"RKP", 50.0},
    {"RAP", -23.13},
    {"RAR", 0.0},
    {"LCY", 0.0},
    {"LCR", 0.0},
    {"LCP", -26.87},
    {"LKP", 50.0},
    {"LAP", -23.13},
    {"LAR", 0.0},
    {"WP",  0.0},
    {"WR",  0.0},
    {"WY",  0.0},
    {"HP",  0.0},
    {"HY",  0.0},
    {"RSC",  0.0},
    {"RSP",  60.0},
    {"RSR",  -20.0},
    {"RSY",  -5.0},
    {"REP",  -105.0},
    {"RWRY",  0.0},
    {"RWRR",  -40.0},
    {"RWRP",  0.0},
    {"RHDY",  0.0},
    {"LSC",  0.0},
    {"LSP",  60.0},
    {"LSR",  20.0},
    {"LSY",  5.0},
    {"LEP",  -105.0},
    {"LWRY",  0.0},
    {"LWRR",  40.0},
    {"LWRP",  0.0},
    {"LHDY",  0.0},
    {"LTMP",  -60.5}, // left thumb
    {"LTPIP",  0.0},
    {"LTDIP",  0.0},
    {"LMMP",  60.5}, // left middle
    {"LMPIP",  0.0},
    {"LMDIP",  0.0},
    {"LIMP",  60.5}, // left index
    {"LIPIP",  0.0},
    {"LIDIP",  0.0},
    {"RTMP",  60.5}, // right thumb   1.0559265
    {"RTPIP",  0.0},
    {"RTDIP",  0.0},
    {"RMMP",  -60.5}, // right middle -1.0559265
    {"RMPIP",  0.0},
    {"RMDIP",  0.0},
    {"RIMP",  -60.5}, // right index -1.0559265
    {"RIPIP",  0.0},
    {"RIDIP", 0.0},
    };
    std::vector<double> halfSittingVector = {};
    std::vector<std::vector<double>> res;
    double joint_value = 0.0f;
    for (const auto &pair : halfSitting)
    {
        joint_value = pair.second;
        halfSittingVector.push_back(joint_value);
    }
    res.push_back(halfSittingVector);
    return res;
}

EXPORT_SINGLE_STATE("Post_Impact_Task", Post_Impact_Task)
