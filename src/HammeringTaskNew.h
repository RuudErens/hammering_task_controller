#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>

// BSplineTrajectoryTask and curve constraints
#include <mc_tasks/PostureTask.h>
#include <ndcurves/curve_constraint.h>


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
    HammeringTaskNew(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;



    bool impact_detected = false;
    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr subForce;
    mc_rtc::NodeHandlePtr nh; 

    // Hammer
    const Eigen::Vector3d normal_vector_to_align_in_hammerhead_frame = {1, 0, 0}; 

    // Nail
    Eigen::Matrix3d nail_rot;
    const Eigen::Vector3d normal_vector_nail_frame = {0, 0, 1};
    Eigen::Vector3d nail_normal_vector_world_frame = {0, 0, 0};
    Eigen::Vector3d nail_force_vector = {0, 0, 0};

    // Logs
    double effective_mass = 0.0f;
    Eigen::Vector3d hammer_tip_actual_velocity_vector = {0, 0, 0};
    Eigen::Vector3d hammer_tip_reference_velocity_vector = {0, 0, 0};
    double projected_momentum_of_hammer_tip = 0.0f;
    double vector_orientation_error = 0.0f;

    std::vector<std::vector<double>> base_posture_vector;


    // ------------------------------ Parameters ---------------------------------------------  
    // Parameters loaded in the load_parameters function, parameters are found in the HammeringTaskNew.in.yaml file
    // Don't ask me why there is a '.in' in the name of the file, I don't know 
    
    mc_rtc::Configuration config_;

    const std::string nail_robot_name = "nail";
    const std::string main_robot_name = "hrp5_p";
    const std::string hammer_head_frame_name = "Hammer_Head";
    const std::string nail_frame_name = "nail";
    
    // quality of life
    
    bool _bezier_curve_verbose_active = false;
    bool _jacobian_verbose_active = false;
    
    // gui
    
    std::string stop_hammering_button_name = "undefined";

    // Minimum force to detect an impact on the nail
    double magic_force_threshold = 1;
    

  private:
    /**
    @brief Loads the parameters found in the HammeringTaskNew.in.yaml file
    */
    void load_parameters();

    /**
    @brief Adds some graphs to the logs of mc_log_ui
     */
    void add_logs();

    /**
    @brief Store the force vector retrieved from the nail sensor plugin
    */
    void nail_force_sensor_callback(const std::shared_ptr<const geometry_msgs::msg::Vector3Stamped> &force);

};