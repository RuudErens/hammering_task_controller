#include "HammeringTaskNew.h"

HammeringTaskNew::HammeringTaskNew(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{

  config_.load(config);
  datastore().make<std::string>("ControlMode", "Torque");
  datastore().make<std::string>("Coriolis", "Yes"); 
  load_parameters();
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
  std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask = getPostureTask(robot().name());
  base_posture_vector = FSMPostureTask->posture();

  mc_rtc::log::success("HammeringTaskNew init done ");
}

bool HammeringTaskNew::run()
{
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);

}

void HammeringTaskNew::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
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
  std::string hammerhead_frame_key = "Hammer_Head";
  std::string nail_frame_key = "nail";
 config_(global_controller)(frames_key)(hammerhead_frame_key, hammer_head_frame_name);
 config_(global_controller)(frames_key)(nail_frame_key, nail_frame_name);

  // ------------------------ Loading magic values ---------------------------

  std::string magic_values_key = "magic_values";
  magic_force_threshold = config_(global_controller)(magic_values_key)("magic_force_threshold");

  

}

void HammeringTaskNew::add_logs()
{
    logger().addLogEntry("Effective mass [kg]", this, [&, this]()
    {return effective_mass;});

    logger().addLogEntry("Hammer tip velocity [m/s]", this, [&, this]()
    {return hammer_tip_actual_velocity_vector;});
      
    logger().addLogEntry("Hammer tip reference bezier velocity [m/s]", this, [&, this]()
    {return hammer_tip_reference_velocity_vector;});
    
    logger().addLogEntry("Projected momentum of hammer tip [kgm/s]", this, [&, this]()
    {return projected_momentum_of_hammer_tip;});

    logger().addLogEntry("Vector orientation error", this, [&, this]()
    {return vector_orientation_error;});
}

