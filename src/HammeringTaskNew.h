#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>

// BSplineTrajectoryTask and curve constraints
#include <mc_solver/DynamicsConstraint.h>
#include <mc_solver/ImpulseConstraint.h>
#include <mc_tasks/PostureTask.h>
#include <ndcurves/curve_constraint.h>

#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>
#include <mc_tasks/lipm_stabilizer/Contact.h>

// Ros node
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "rclcpp/rclcpp.hpp" //including ros2
#include <mc_rtc_ros/ros.h>
#include <vector>
#include "api.h"


typedef Eigen::Vector3d Point;
typedef Point point_t;
typedef ndcurves::curve_constraints<point_t> curve_constraints_t;

struct HammeringTaskNew_DLLAPI HammeringTaskNew : public mc_control::fsm::Controller
{
public:
    HammeringTaskNew(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration& config);

    bool run() override;

    void reset(const mc_control::ControllerResetData& reset_data) override;

    /**
     @brief Compute the effective mass of the hammerhead in the normal direction of the nail
     */
    const double compute_effective_mass_with_mbc();

    // ROS subscriber to the nail force sensor topic
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr subForce;
    mc_rtc::NodeHandlePtr nh;

    // Hammer head constants
    const Eigen::Vector3d normal_vector_to_align_in_hammerhead_frame = {1, 0, 0};

    // Nail
    Eigen::Matrix3d nail_rot;
    const Eigen::Vector3d normal_vector_nail_local = {0, 0, 1}; // Setting the normal vector of the nail to be along the z-axis of the nail frame
    Eigen::Vector3d nail_normal_vector_world_frame = {0, 0, 0};
    Eigen::Vector3d nail_force_vector = {0, 0, 0};

    // Variables for logging
    double effective_mass = 0.0f;
    double effective_mass_diff = 0.0f;
    double effective_mass_diff_diff = 0.0f;
    double eff_mass_diff_checker = 0.f;
    Eigen::Vector3d hammer_tip_actual_velocity_vector = {0, 0, 0};
    Eigen::Vector3d hammer_tip_actual_position_vector = {0, 0, 0};
    Eigen::Vector3d hammer_tip_actual_position_vector_realrobot = {0, 0, 0};
    Eigen::Vector3d hammer_tip_reference_velocity_vector = {0, 0, 0};
    Eigen::Vector3d hammer_tip_reference_position_vector = {0, 0, 0};
    Eigen::Vector3d hammer_tip_position_observer_error = {0, 0, 0};
    Eigen::Vector3d floating_base_position_observer_error = {0, 0, 0};

    Eigen::Vector3d bspline_tracking_error = {0, 0, 0};
    Eigen::Vector6d bspline_eval = {0, 0, 0, 0, 0, 0};
    double bspline_eval_norm = 0.0f;
    bool bspline_active_ = false;

    double projected_momentum_of_hammer_tip = 0.0f;

    double vector_orientation_error = 0.0f;

    int trajectories_executed = 0;

    double stabilizing_eval_norm = 0.0f;
    double stabilizing_speed_norm = 0.0f;

    // Hitting quality logging
    double last_hitting_angle = 0.0f;
    double last_projected_momentum_of_hammer_tip = 0.0f;
    double last_hitting_angle_bodysensor = 0.0f;
    Eigen::Vector3d last_hitting_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_hitting_point_error_tilt = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_hitting_point_error_bodysensor = Eigen::Vector3d::Zero();
    double previous_hitting_angle = 0.0f;
    double previous_projected_momentum_of_hammer_tip = 0.0f;
    double previous_hitting_angle_bodysensor = 0.0f;
    Eigen::Vector3d previous_hitting_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_hitting_point_error_tilt = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_hitting_point_error_bodysensor = Eigen::Vector3d::Zero();
    bool hitting_data_to_log = false;
    bool hitting_logging_entry_to_remove = false;
    bool force_felt = false;
    int number_of_hits = 0;

    // Constraints and their parameters
    std::array<double, 3> damping;
    double vp;
    std::unique_ptr<mc_solver::DynamicsConstraint> dynamicsConstraint;

    double c_res;
    double lambda_high;
    double lambda_low;
    double impact_duration;
    double impulsive_torque_limit_multiplier;
    std::unique_ptr<mc_solver::ImpulseConstraint> impulseConstraint;

    std::unique_ptr<mc_solver::ContactConstraint> contactConstraintSet;

    // Tasks and their parameters
    std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> stabilizerTask;
    mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stabiConf;
    double torso_task_stiffness = 1.0f;
    double torso_task_weight = 1.0f;
    double pelvis_task_stiffness = 1.0f;
    double pelvis_task_weight = 1.0f;
    Eigen::Vector2d dcm_p = Eigen::Vector2d::Zero();
    Eigen::Vector2d dcm_i = Eigen::Vector2d::Zero();
    Eigen::Vector2d dcm_d = Eigen::Vector2d::Zero();
    Eigen::Vector3d com_stiffness = Eigen::Vector3d::Zero();
    double com_weight = 1.0f;
    double contact_task_weight = 1.0f;
    sva::MotionVecd contact_stiffness = sva::MotionVecd::Zero();
    sva::MotionVecd contact_damping = sva::MotionVecd::Zero();
    Eigen::Vector2d contact_admittance = Eigen::Vector2d::Zero();

    double base_posture_weight = 1.0f;
    double base_posture_stiffness = 1.0f;

    // Parameter loader
    mc_rtc::Configuration config_;

    const std::string nail_robot_name = "nail";
    const std::string main_robot_name = "hrp5_p";
    const std::string hammer_head_frame_name = "Hammer_head";
    const std::string nail_frame_name = "nail";

    // gui
    std::string stop_hammering_button_name = "undefined";

    // Minimum force to detect an impact on the nail
    double impact_detection_force_threshold = 1;
    bool impact_detected = false;

    // Robot copy to determine the error off the Til observer, TODO: Take out for experiments on real robot
    std::shared_ptr<mc_rbdyn::Robots> comparisonRobots_;
    // Bodysensor to determing the error of the Tilt observer, TODO: Take out for experiments on real robot
    const mc_rbdyn::BodySensor& floatingBaseSensor_ = robot().bodySensor("FloatingBase");

    int max_number_of_hits = 0;

private:
    /**
    @brief Loads the parameters found in the configuration yaml-file (inputs in etc/HammeringTaskNew.in.yaml)
    */
    void load_parameters();

    /**
    @brief Adds variables to the logs of the controller
     */
    void add_logs();

    /**
    @brief Store the force vector retrieved from the nail sensor plugin
    */
    void nail_force_sensor_callback(const std::shared_ptr<const geometry_msgs::msg::Vector3Stamped>& force);
};
