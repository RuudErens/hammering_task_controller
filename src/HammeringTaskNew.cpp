#include "HammeringTaskNew.h"
#include <RBDyn/MultiBodyConfig.h>
// #include <mc_solver/DynamicsConstraint.h>


HammeringTaskNew::HammeringTaskNew(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration& config)
    : mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
    config_.load(config);
    datastore().make<std::string>("ControlMode", "Torque");
    datastore().make<std::string>("Coriolis", "Yes");

    // Load parameters from config yaml-file
    load_parameters();

    // Set variables to be logged
    add_logs();

    // Set the ROS subscriber to the nail force sensor topic
    nh = mc_rtc::ROSBridge::get_node_handle();
    if (nh != nullptr)
    {
        subForce = nh->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/nail_force_sensor",
            1000,
            std::bind(&HammeringTaskNew::nail_force_sensor_callback, this, std::placeholders::_1));
    }

    // Set the nail rotation matrix and world-frame normal vector to be used in any later state
    nail_rot = robot(nail_robot_name).frame(nail_frame_name).position().rotation();
    nail_normal_vector_world_frame = (nail_rot.transpose() * normal_vector_nail_local).normalized();

    // Set contact constraints for the feet
    contactConstraintSet = std::make_unique<mc_solver::ContactConstraint>(
        timeStep, mc_solver::ContactConstraint::ContactType::Acceleration);
    solver().addConstraintSet(contactConstraintSet);
    addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
    addContact({robot().name(), "ground", "RightFoot", "AllGround"});

    // Set dynamics constraint
    dynamicsConstraint = std::make_unique<mc_solver::DynamicsConstraint>(
        robots(), robot().robotIndex(), solver().dt(), damping, vp, false, true);
    solver().addConstraintSet(dynamicsConstraint);

    // Create impulse constraint to be added in specific states
    Eigen::Vector3d normal_nail = robot(nail_robot_name).frame(nail_frame_name).position().rotation().col(2).eval();
    impulseConstraint = std::make_unique<mc_solver::ImpulseConstraint>(robots(), robot().robotIndex(),
                                                                       robot().frame(hammer_head_frame_name),
                                                                       normal_nail, lambda_high, lambda_low, impact_duration,
                                                                       c_res, impulsive_torque_limit_multiplier, logger());

    // Load default configuration of stabilizer taskfrom robot module
    stabiConf = robot().module().defaultLIPMStabilizerConfiguration();
    stabiConf.copMaxVel = {{3., 3., 3.}, {0.1, 0.1, 0.1}};
    // Create the stabilizer task, configure it and add it to the solver
    stabilizerTask = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
        solver().robots(),
        solver().realRobots(),
        robot().robotIndex(),
        stabiConf.leftFootSurface,
        stabiConf.rightFootSurface,
        stabiConf.torsoBodyName,
        solver().dt());
    stabilizerTask->reset();
    stabilizerTask->configure(stabiConf);
    stabilizerTask->torsoStiffness(torso_task_stiffness);
    stabilizerTask->torsoWeight(torso_task_weight);
    stabilizerTask->pelvisStiffness(pelvis_task_stiffness);
    stabilizerTask->pelvisWeight(pelvis_task_weight);
    stabilizerTask->dcmGains(dcm_p, dcm_i, dcm_d);
    stabilizerTask->comStiffness(com_stiffness);
    stabilizerTask->comWeight(com_weight);
    stabilizerTask->contactWeight(contact_task_weight);
    stabilizerTask->contactStiffness(contact_stiffness);
    stabilizerTask->contactDamping(contact_damping);
    stabilizerTask->copAdmittance(contact_admittance);
    solver().addTask(stabilizerTask);

    mc_rtc::log::success("HammeringTaskNew init done ");
}

bool HammeringTaskNew::run()
{
    // Update the state of the comparison robot to match the real robot, to be able to measure the observer error of the Tilt observer
    comparisonRobots_->robot().mbc().q = realRobot().mbc().q;
    // Manually set the floating base pose, velocity and acceleration in the world frame from the bodysensor
    comparisonRobots_->robot().posW(sva::PTransformd(floatingBaseSensor_.orientation(),
                                                      floatingBaseSensor_.position()));
    comparisonRobots_->robot().velW(sva::MotionVecd(floatingBaseSensor_.angularVelocity(),
                                                    floatingBaseSensor_.linearVelocity()));
    comparisonRobots_->robot().accW(sva::MotionVecd(floatingBaseSensor_.angularAcceleration(),
                                                    floatingBaseSensor_.linearAcceleration()));

    // Values to se the tracking performance hammering motion and the observer
    hammer_tip_actual_position_vector_realrobot = comparisonRobots_->robot().frame(hammer_head_frame_name).position().
                                                                     translation();
    hammer_tip_actual_position_vector = robot().frame(hammer_head_frame_name).position().translation();
    hammer_tip_actual_velocity_vector = robot().frame(hammer_head_frame_name).velocity().linear();

    hammer_tip_position_observer_error = hammer_tip_actual_position_vector -
        hammer_tip_actual_position_vector_realrobot;
    floating_base_position_observer_error = comparisonRobots_->robot().posW().translation() - robot().posW().
        translation();

    // Stabilizing task stats which can be used to check stabilization state (based on the com task and the contacts)
    Eigen::VectorXd vec_joined(stabilizerTask->comeval().size() + stabilizerTask->contacteval().size());
    vec_joined << stabilizerTask->comeval(), stabilizerTask->contacteval();
    stabilizing_eval_norm = vec_joined.norm();
    stabilizing_speed_norm = stabilizerTask->speed().norm();

    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
}

void HammeringTaskNew::reset(const mc_control::ControllerResetData& reset_data)
{
    // Setup the copmarison robot for observer error measurement of the tilt observer
    comparisonRobots_ = mc_rbdyn::loadRobot(robot().module());
    mc_control::fsm::Controller::reset(reset_data);
}

const double HammeringTaskNew::compute_effective_mass_with_mbc()
{
    // update the mbc for M and J
    rbd::MultiBodyConfig mbc = robot().mbc();
    robot().forwardKinematics(robot().mbc());

    //  Access Full Jacobian of the robot
    rbd::MultiBody robot_mb = robot().mb();
    rbd::Jacobian jac(robot_mb, hammer_head_frame_name);
    Eigen::MatrixXd world_frame_jacobian = jac.bodyJacobian(robot_mb, mbc);
    Eigen::MatrixXd full_world_frame_jacobian(6, robot().mb().nrDof());
    jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);

    // Access linear part of the Jacobian
    const Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);

    // Access Mass matrix
    rbd::ForwardDynamics fd(robot_mb);
    fd.computeH(robot_mb, mbc);
    Eigen::MatrixXd M = fd.H();

    // Compute Lambda
    const Eigen::Matrix3d LAMBDA = linear_jacobian * M.inverse() * linear_jacobian.transpose();

    return 1 / (nail_normal_vector_world_frame.transpose() * LAMBDA * nail_normal_vector_world_frame);
} // TODO: maybe generalize this function for both Get_In_Position_Task as Post_Impact_Task

void HammeringTaskNew::nail_force_sensor_callback(
    const std::shared_ptr<const geometry_msgs::msg::Vector3Stamped>& force)
{
    nail_force_vector.x() = force->vector.x;
    nail_force_vector.y() = force->vector.y;
    nail_force_vector.z() = force->vector.z;
}

void HammeringTaskNew::load_parameters()
{
    std::string global_controller = "global_controller_params";

    // ------------------------ Loading gui parameters ---------------------------

    std::string gui_key = "gui";
    std::string stop_hammering_button_name_key = "stop_hammering_button_name";
    config_(global_controller)(gui_key)(stop_hammering_button_name_key, stop_hammering_button_name);

    // ------------------------ Loading frames ---------------------------

    std::string frames_key = "frames";
    std::string hammerhead_frame_key = "Hammer_head";
    std::string nail_frame_key = "nail";
    config_(global_controller)(frames_key)(hammerhead_frame_key, hammer_head_frame_name);
    config_(global_controller)(frames_key)(nail_frame_key, nail_frame_name);

    // ------------------------ Loading state parameters ---------------------------

    std::string tuning_params_key = "parameters";
    impact_detection_force_threshold = config_(global_controller)(tuning_params_key)("impact_detection_force_threshold");
    max_number_of_hits = config_(global_controller)(tuning_params_key)("max_number_of_hits");

    // ------------------------ Loading constraint parameters ---------------------------

    // Parameters for the impulse constraint
    c_res = config_(global_controller)(tuning_params_key)("c_res");
    lambda_high = config_(global_controller)(tuning_params_key)("lambda_high");
    lambda_low = config_(global_controller)(tuning_params_key)("lambda_low");
    impact_duration = config_(global_controller)(tuning_params_key)("delta_t");
    impulsive_torque_limit_multiplier = config_(global_controller)(tuning_params_key)("impulsive_tau_limit_multiplier");

    // Parameters for the dynamics constraint
    damping = config_(global_controller)(tuning_params_key)("damping");
    vp = config_(global_controller)(tuning_params_key)("velocity_percentage");

    // ------------------------ Loading base parameters ---------------------------
    std::string robot_key = "hrp5_p";
    std::string posture_key = "posture";
    base_posture_stiffness = config_(robot_key)(posture_key)("stiffness");
    base_posture_weight = config_(robot_key)(posture_key)("weight");

    // ------------------------ Loading stabilizer parameters ---------------------------
    std::string global_control_param_key = "global_controller_params";
    std::string stabilizer_key = "stabilizer";

    torso_task_stiffness = config_(global_control_param_key)(stabilizer_key)("torso")("stiffness");
    torso_task_weight = config_(global_control_param_key)(stabilizer_key)("torso")("weight");
    pelvis_task_stiffness = config_(global_control_param_key)(stabilizer_key)("pelvis")("stiffness");
    pelvis_task_weight = config_(global_control_param_key)(stabilizer_key)("pelvis")("weight");
    dcm_p = config_(global_control_param_key)(stabilizer_key)("dcm")("p");
    dcm_i = config_(global_control_param_key)(stabilizer_key)("dcm")("i");
    dcm_d = config_(global_control_param_key)(stabilizer_key)("dcm")("d");
    com_stiffness = config_(global_control_param_key)(stabilizer_key)("com")("stiffness");
    com_weight = config_(global_control_param_key)(stabilizer_key)("com")("weight");
    contact_task_weight = config_(global_control_param_key)(stabilizer_key)("contact")("weight");
    contact_stiffness = config_(global_control_param_key)(stabilizer_key)("contact")("stiffness");
    contact_damping = config_(global_control_param_key)(stabilizer_key)("contact")("damping");
    contact_admittance = config_(global_control_param_key)(stabilizer_key)("contact")("admittance");
}

void HammeringTaskNew::add_logs()
{
    logger().addLogEntry("Effective mass [kg]", this, [&, this]()
    {
        return effective_mass;
    });

    logger().addLogEntry("Effective mass derivative", this, [&, this]()
    {
        return effective_mass_diff;
    });

    logger().addLogEntry("Effective mass double derivative", this, [&, this]()
    {
        return effective_mass_diff_diff;
    });

    logger().addLogEntry("Effective mass diff checker", this, [&, this]()
    {
        return eff_mass_diff_checker;
    });

    logger().addLogEntry("Hammer tip velocity [m/s]", this, [&, this]()
    {
        return hammer_tip_actual_velocity_vector;
    });

    logger().addLogEntry("Hammer tip position [m]", this, [&, this]()
    {
        return hammer_tip_actual_position_vector;
    });

    logger().addLogEntry("Hammer tip position real robot [m]", this, [&, this]()
    {
        return hammer_tip_actual_position_vector_realrobot;
    });

    logger().addLogEntry("floating base body sensor_position", this, [&, this]()
    {
        return floatingBaseSensor_.position();
    });

    logger().addLogEntry("floating base body sensor_orientation", this, [&, this]()
    {
        return floatingBaseSensor_.orientation();
    });

    logger().addLogEntry("floating base body sensor_linearvelocity", this, [&, this]()
    {
        return floatingBaseSensor_.linearVelocity();
    });

    logger().addLogEntry("floating base body sensor_angularvelocity", this, [&, this]()
    {
        return floatingBaseSensor_.angularVelocity();
    });

    logger().addLogEntry("floating base body sensor_linearacceleration", this, [&, this]()
    {
        return floatingBaseSensor_.linearAcceleration();
    });

    logger().addLogEntry("floating base body sensor_angularacceleration", this, [&, this]()
    {
        return floatingBaseSensor_.angularAcceleration();
    });

    logger().addLogEntry("floating base observer error position", this, [&, this]()
    {
        return floating_base_position_observer_error;
    });

    logger().addLogEntry("Hammer tip observer error position", this, [&, this]()
    {
        return hammer_tip_position_observer_error;
    });

    logger().addLogEntry("Projected momentum of hammer tip [kgm/s]", this, [&, this]()
    {
        return projected_momentum_of_hammer_tip;
    });

    logger().addLogEntry("Nail force sensor", this, [&, this]()
    {
        return nail_force_vector;
    });

    logger().addLogEntry("Nail force sensor norm", this, [&, this]()
    {
        return nail_force_vector.norm();
    });

    logger().addLogEntry("bspline_active", this, [&, this]()
    {
        return 100 * bspline_active_;
    });

    logger().addLogEntry("Stabilizing_speed_norm", this, [&, this]()
    {
        return stabilizing_speed_norm;
    });

    logger().addLogEntry("Stabilizing_eval_norm", this, [&, this]()
    {
        return stabilizing_eval_norm;
    });

    logger().addLogEntry("Completed_trajectories", this, [&, this]()
    {
        return trajectories_executed;
    });
}
