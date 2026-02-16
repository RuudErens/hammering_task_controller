#include "HammeringTaskNew.h"
#include <RBDyn/MultiBodyConfig.h>
// #include <mc_solver/DynamicsConstraint.h>


HammeringTaskNew::HammeringTaskNew(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{


  config_.load(config);
  datastore().make<std::string>("ControlMode", "Position");
  datastore().make<std::string>("Coriolis", "Yes"); 
  load_parameters();

  // effective_mass = compute_effective_mass_with_mbc();
  add_logs();
  nh = mc_rtc::ROSBridge::get_node_handle();
  // Not the cleanest but at leat mc_mujoco does not crash
  if(nh != nullptr)
  {
    subForce = nh->create_subscription<geometry_msgs::msg::Vector3Stamped>(
                  "/nail_force_sensor", 
                  1000,
                  std::bind(&HammeringTaskNew::nail_force_sensor_callback, this, std::placeholders::_1));
  }
  nail_rot = robot(nail_robot_name).frame(nail_frame_name).position().rotation();

  // Nail normal vector (n) expressed in world frame 
  nail_normal_vector_world_frame = (nail_rot.transpose()*normal_vector_nail_frame).normalized();

  // Store the initial posture of the robot
  // solver().addTask(postureTask);
  // postureTask->stiffness(100);
  contactConstraintSet = std::make_unique<mc_solver::ContactConstraint>(timeStep, mc_solver::ContactConstraint::ContactType::Acceleration);
  solver().addConstraintSet(contactConstraintSet);
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});

  // std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask = getPostureTask(robot().name());
  // base_posture_vector = FSMPostureTask->posture();

    // dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    //   new mc_solver::DynamicsConstraint(
    //       robots(), 0, {0.1, 0.01, xsiOff_, m_, lambda_}, 0.9, true));
  // const std::array<double, 3> damping = {0.55, 0.30, 0.9};
  dynamicsConstraint = std::make_unique<mc_solver::DynamicsConstraint>(robots(), robot().robotIndex(), solver().dt(), _damping, _vp, false, true);
  solver().addConstraintSet(dynamicsConstraint);

  // Add impulse constraint
  Eigen::Vector3d normal_nail = robot(nail_robot_name).frame(nail_frame_name).position().rotation().col(2).eval();
  impulseConstraint = std::make_unique<mc_solver::ImpulseConstraint>(robots(), robot().robotIndex(), robot().frame(hammer_head_frame_name), normal_nail, _lambda_high, _lambda_low, _delta_t, _c_res, _dt_multi, logger());
  // solver().addConstraintSet(impulseConstraint);

  // Print the joint names of the jionts in the q vector
  // mc_rtc::log::info("the robot has {} joints", robot().mb().nrJoints());
  // for (int i=0; i<robot().mb().nrJoints(); ++i)
  // {
  //   const rbd::Joint & joint = robot().mb().joint(i);
  //   for (size_t j=0; j<joint.dof(); ++j)
  //   {
  //     mc_rtc::log::info("{}", joint.name());
  //   }
  // }
  //
  // for (auto frame : robot().frames())
  // {
  //   mc_rtc::log::info("Frame {} is in {}", frame, robot().name());
  // }

  // mc_rtc::log::info(robot().tvmRobot().limits().tu);

  // controller->robots().robot(r.name).module().ref_joint_order()
  // // Add arrow to check whether we use the correct normal vector
  // start_ = robot(nail_robot_name).frame(nail_frame_name).position().translation();
  // end_ = start_ + 0.2*robot(nail_robot_name).frame(nail_frame_name).position().rotation().col(2).eval();
  // gui()->addElement({"a", "b"},
  // mc_rtc::gui::Arrow("ArrowRO", [this]() { return start_; }, [this]() { return end_; })
  // );

  // Load default configuration from robot module
  stabiConf = robot().module().defaultLIPMStabilizerConfiguration();
  stabiConf.copMaxVel = {{3., 3., 3.}, {0.1, 0.1, 0.1}};
  // Create the stabilizer task
  stabilizerTask = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
            solver().robots(),
            solver().realRobots(),
            robot().robotIndex(),
            stabiConf.leftFootSurface,
            stabiConf.rightFootSurface,
            stabiConf.torsoBodyName,
            solver().dt());
  // Reset the task targets and default configuration
  stabilizerTask->reset();
  // Apply stabilizer configuration (optional, if not provided the default configuration from the RobotModule will be used)
  stabilizerTask->configure(stabiConf);
  // Set contacts (optional, the stabilizer will be configured in double support using the current foot pose as target for each contact by default)
  // stabilizerTask->setContacts({ContactState::Left, ContactState::Right});
  solver().addTask(stabilizerTask);
  stabilizerTask->torsoStiffness(_torso_task_stiffness);
  stabilizerTask->torsoWeight(_torso_task_weight);
  stabilizerTask->pelvisStiffness(_pelvis_task_stiffness);
  stabilizerTask->pelvisWeight(_pelvis_task_weight);
  stabilizerTask->dcmGains(_dcm_p, _dcm_i, _dcm_d);
  stabilizerTask->comStiffness(_com_stiffness);
  stabilizerTask->comWeight(_com_weight);
  stabilizerTask->contactWeight(_contact_task_weight);
  stabilizerTask->contactStiffness(_contact_stiffness);
  stabilizerTask->contactDamping(_contact_damping);
  stabilizerTask->copAdmittance(_contact_admittance);

  auto ext_wrench_conf = stabilizerTask->externalWrenchConfiguration();
  ext_wrench_conf.addExpectedCoMOffset = true;
  ext_wrench_conf.modifyCoMErr = true;
  ext_wrench_conf.modifyZMPErr = true;
  stabilizerTask->externalWrenchConfiguration(ext_wrench_conf);

  stabiConf = stabilizerTask->config();

  auto stab_config = stabilizerTask->config();

  auto & Active_tasks = solver().tasks();
  for (auto i:Active_tasks){
    mc_rtc::log::info("This controller has task: {} of type: {}", i->name(), i->type());
  }

  mc_rtc::log::success("HammeringTaskNew init done ");
}

bool HammeringTaskNew::run()
{
  // auto & bsRobot = comparisonRobots_->robot();
  // 1. Copy joint positions from the real robot
  // bsRobot.encoderValues(realRobot().encoderValues());

  // 2. Get the BodySensor data
  // Replace "FloatingBase" with the actual name of the sensor in your robot module
  // auto available_sensors = robot().bodySensors();
  // mc_rtc::log::info("The following bodysensors are available:");
  // for (const auto & sensor : available_sensors)
  // {
  //   mc_rtc::log::info(" - {}", sensor.name());
  // }
  // const auto & sensor = robot().bodySensor("FloatingBase");

  comparisonRobots_->robot().mbc().q = realRobot().mbc().q;

  // Manually set the floating base pose, velocity and acceleration in the world frame from the bodysensor
  comparisonRobots_->robot().posW(sva::PTransformd(floatingBaseSensor_.orientation(), floatingBaseSensor_.position()));
  comparisonRobots_->robot().velW(sva::MotionVecd(floatingBaseSensor_.angularVelocity(), floatingBaseSensor_.linearVelocity()));
  comparisonRobots_->robot().accW(sva::MotionVecd(floatingBaseSensor_.angularAcceleration(), floatingBaseSensor_.linearAcceleration()));

  // // Update the kinematic tree
  // comparisonRobots_->robot().forwardKinematics();
  // comparisonRobots_->robot().forwardVelocity();
  // comparisonRobots_->robot().forwardAcceleration();

  hammer_tip_actual_position_vector_realrobot = comparisonRobots_->robot().frame(hammer_head_frame_name).position().translation();

  hammer_tip_actual_position_vector = robot().frame(hammer_head_frame_name).position().translation();
  hammer_tip_actual_velocity_vector = robot().frame(hammer_head_frame_name).velocity().linear();

  hammer_tip_position_observer_error = hammer_tip_actual_position_vector - hammer_tip_actual_position_vector_realrobot;
  floating_base_position_observer_error = comparisonRobots_->robot().posW().translation() - robot().posW().translation();

  Eigen::VectorXd vec_joined(stabilizerTask->comeval().size() + stabilizerTask->contacteval().size());
  vec_joined << stabilizerTask->comeval(), stabilizerTask->contacteval();
  stabilizing_eval_norm = vec_joined.norm();
  stabilizing_speed_norm = stabilizerTask->speed().norm();

  com_eval = stabilizerTask->comeval();
  pelvis_eval = stabilizerTask->pelviseval();
  torso_eval = stabilizerTask->torsoeval();
  contacts_eval = stabilizerTask->contacteval();
  // contacts_eval = Eigen::VectorXd::Zero(6);

  com_eval_norm = com_eval.norm();
  pelvis_eval_norm = pelvis_eval.norm();
  torso_eval_norm = torso_eval.norm();
  contacts_eval_norm = contacts_eval.norm();

  // com_eval_norm = 0;
  // pelvis_eval_norm = 0;
  // torso_eval_norm = 0;
  // contacts_eval_norm = 0;

  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop); // TODO: set to closedloop

}

void HammeringTaskNew::reset(const mc_control::ControllerResetData & reset_data)
{
  // auto robots = mc_rbdyn::loadRobot(robot().module());
  comparisonRobots_ = mc_rbdyn::loadRobot(robot().module());
  // comparisonRobots_ = std::make_shared<mc_rbdyn::Robot>(robots->robot(0).module(), robots->robot(0).name());
  mc_control::fsm::Controller::reset(reset_data);
}

const double HammeringTaskNew::compute_effective_mass_with_mbc(){
    
  // If you dont put this line the gradient is 0 everywhere because M and J are not updating
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
  const Eigen::Matrix3d LAMBDA = linear_jacobian*M.inverse()*linear_jacobian.transpose();

  return 1/(nail_normal_vector_world_frame.transpose()*LAMBDA*nail_normal_vector_world_frame);

}


void HammeringTaskNew::nail_force_sensor_callback(const std::shared_ptr<const geometry_msgs::msg::Vector3Stamped> &force)
{

  nail_force_vector.x() = force->vector.x;
  nail_force_vector.y() = force->vector.y;
  nail_force_vector.z() = force->vector.z;

}

void HammeringTaskNew::load_parameters()
{
  std::string global_controller = "global_controller_params";
  // ------------------------ Loading timestep ---------------------------
  std::string timestep_key = "timestep";

  // ------------------------ Loading gui parameters ---------------------------

  std::string gui_key = "gui";
  std::string stop_hammering_button_name_key = "stop_hammering_button_name";
  config_(global_controller)(gui_key)(stop_hammering_button_name_key, stop_hammering_button_name);

  // ------------------------ Loading quality of life parameters ---------------------------

  std::string qol_key = "quality_of_life";
  std::string jacobian_verbose_active_key = "jacobian_verbose_active";
  std::string bezier_curve_verbose_active_key = "bezier_curve_verbose_active";
  _bezier_curve_verbose_active = config_(global_controller)(qol_key)(bezier_curve_verbose_active_key);
  _jacobian_verbose_active = config_(global_controller)(qol_key)(jacobian_verbose_active_key);



  // ------------------------ Loading frames ---------------------------

  std::string frames_key = "frames";
  std::string hammerhead_frame_key = "Hammer_head";
  std::string nail_frame_key = "nail";
 config_(global_controller)(frames_key)(hammerhead_frame_key, hammer_head_frame_name);
 config_(global_controller)(frames_key)(nail_frame_key, nail_frame_name);

  // ------------------------ Loading magic values ---------------------------

  std::string magic_values_key = "magic_values";
  magic_force_threshold = config_(global_controller)(magic_values_key)("magic_force_threshold");
  max_number_of_hits = config_(global_controller)(magic_values_key)("max_number_of_hits");

  // ------------------------ Loading constraint parameters ---------------------------
  _c_res  = config_(global_controller)(magic_values_key)("c_res");
  _lambda_high  = config_(global_controller)(magic_values_key)("lambda_high");
  _lambda_low = config_(global_controller)(magic_values_key)("lambda_low");
  _delta_t  = config_(global_controller)(magic_values_key)("delta_t");
  _dt_multi  = config_(global_controller)(magic_values_key)("impulsive_tau_limit_multiplier");
  _damping  = config_(global_controller)(magic_values_key)("damping");
  _vp  = config_(global_controller)(magic_values_key)("velocity_percentage");

  // ------------------------ Loading base parameters ---------------------------
  std::string robot_key = "hrp5_p";
  std::string posture_key = "posture";
  base_posture_stiffness = config_(robot_key)(posture_key)("stiffness");
  base_posture_weight = config_(robot_key)(posture_key)("weight");

  // ------------------------ Loading stabilizer parameters ---------------------------
  std::string global_control_param_key = "global_controller_params";
  std::string stabilizer_key = "stabilizer";

  _torso_task_stiffness = config_(global_control_param_key)(stabilizer_key)("torso")("stiffness");
  _torso_task_weight = config_(global_control_param_key)(stabilizer_key)("torso")("weight");
  _pelvis_task_stiffness = config_(global_control_param_key)(stabilizer_key)("pelvis")("stiffness");
  _pelvis_task_weight = config_(global_control_param_key)(stabilizer_key)("pelvis")("weight");
  _dcm_p = config_(global_control_param_key)(stabilizer_key)("dcm")("p");
  _dcm_i = config_(global_control_param_key)(stabilizer_key)("dcm")("i");
  _dcm_d = config_(global_control_param_key)(stabilizer_key)("dcm")("d");
  _com_stiffness = config_(global_control_param_key)(stabilizer_key)("com")("stiffness");
  _com_weight = config_(global_control_param_key)(stabilizer_key)("com")("weight");
  _contact_task_weight = config_(global_control_param_key)(stabilizer_key)("contact")("weight");
  _contact_stiffness = config_(global_control_param_key)(stabilizer_key)("contact")("stiffness");
  _contact_damping = config_(global_control_param_key)(stabilizer_key)("contact")("damping");
  _contact_admittance = config_(global_control_param_key)(stabilizer_key)("contact")("admittance");



}

void HammeringTaskNew::add_logs()
{
    logger().addLogEntry("Effective mass [kg]", this, [&, this]()
    {return effective_mass;});

    logger().addLogEntry("Effective mass diff", this, [&, this]()
    {return effective_mass_diff;});

    logger().addLogEntry("Effective mass diff checker", this, [&, this]()
    {return eff_mass_diff_checker;});

    logger().addLogEntry("Hammer tip velocity [m/s]", this, [&, this]()
    {return hammer_tip_actual_velocity_vector;});
      
    // logger().addLogEntry("Hammer tip reference bezier velocity [m/s]", this, [&, this]()
    // {return hammer_tip_reference_velocity_vector;});

    logger().addLogEntry("Hammer tip position [m]", this, [&, this]()
    {return hammer_tip_actual_position_vector;});

    logger().addLogEntry("Hammer tip position real robot [m]", this, [&, this]()
    {return hammer_tip_actual_position_vector_realrobot;});

    logger().addLogEntry("floating base body sensor_position", this, [&, this]()
    {return floatingBaseSensor_.position();});

    logger().addLogEntry("floating base body sensor_orientation", this, [&, this]()
    {return floatingBaseSensor_.orientation();});

    logger().addLogEntry("floating base body sensor_linearvelocity", this, [&, this]()
    {return floatingBaseSensor_.linearVelocity();});

    logger().addLogEntry("floating base body sensor_angularvelocity", this, [&, this]()
    {return floatingBaseSensor_.angularVelocity();});

    logger().addLogEntry("floating base body sensor_linearacceleration", this, [&, this]()
    {return floatingBaseSensor_.linearAcceleration();});

    logger().addLogEntry("floating base body sensor_angularacceleration", this, [&, this]()
    {return floatingBaseSensor_.angularAcceleration();});

    logger().addLogEntry("floating base observer error position", this, [&, this]()
    {return floating_base_position_observer_error;});

    logger().addLogEntry("Hammer tip observer error position", this, [&, this]()
    {return hammer_tip_position_observer_error;});

    // logger().addLogEntry("Hammer tip reference bezier position [m]", this, [&, this]()
    // {return hammer_tip_reference_position_vector;});

    // logger().addLogEntry("Bspline tracking error [m]", this, [&, this]()
    // {return bspline_tracking_error;});

    logger().addLogEntry("Projected momentum of hammer tip [kgm/s]", this, [&, this]()
    {return projected_momentum_of_hammer_tip;});

    // logger().addLogEntry("Vector orientation error", this, [&, this]()
    // {return vector_orientation_error;});

    logger().addLogEntry("Nail force sensor", this, [&, this]()
    {return nail_force_vector;});

    logger().addLogEntry("Nail force sensor norm", this, [&, this]()
    {return nail_force_vector.norm();});

    // logger().addLogEntry("Normal force applied to the nail", this, [&, this]()
    // {return vector_orientation_error;});

    logger().addLogEntry("bspline_active", this, [&, this]()
    {return 100*bspline_active_;});

    logger().addLogEntry("Stabilizing_speed_norm", this, [&, this]()
    {return stabilizing_speed_norm;});

    logger().addLogEntry("Stabilizing_eval_norm", this, [&, this]()
    {return stabilizing_eval_norm;});

    logger().addLogEntry("Stabilizing_com_eval_norm", this, [&, this]()
    {return com_eval_norm;});

    logger().addLogEntry("Stabilizing_torso_eval_norm", this, [&, this]()
    {return torso_eval_norm;});

    logger().addLogEntry("Stabilizing_pelvis_eval_norm", this, [&, this]()
    {return pelvis_eval_norm;});

    logger().addLogEntry("Stabilizing_contact_eval_norm", this, [&, this]()
    {return contacts_eval_norm;});

    logger().addLogEntry("Stabilizing_com_eval", this, [&, this]()
    {return com_eval;});

    logger().addLogEntry("Stabilizing_torso_eval", this, [&, this]()
    {return torso_eval;});

    logger().addLogEntry("Stabilizing_pelvis_eval", this, [&, this]()
    {return pelvis_eval;});

    logger().addLogEntry("Stabilizing_contact_eval", this, [&, this]()
    {return contacts_eval;});

    logger().addLogEntry("Completed_trajectories", this, [&, this]()
    {return trajectories_executed;});

}

