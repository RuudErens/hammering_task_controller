// Written by Jimmy VU

#pragma once
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <cstdint>
#include <mc_control/fsm/State.h>

#include <Eigen/src/Geometry/Quaternion.h>
#include <SpaceVecAlg/EigenTypedef.h>
#include <SpaceVecAlg/MotionVec.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_tasks/PositionTask.h>
#include <mc_trajectory/BSpline.h>

#include <memory>
#include <ndcurves/curve_constraint.h>
#include <string>

// Ros node
// #include <geometry_msgs/msg/vector3_stamped.hpp>
// #include "rclcpp/rclcpp.hpp" //including ros2
#include <mc_rtc_ros/ros.h>
#include <thread>

#include <RBDyn/Jacobian.h>
#include <vector>

#include <mc_tasks/VectorOrientationTask.h>

struct Get_In_Position_Task : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  private:

    // Needed to access the mass matrix 
    std::unique_ptr<mc_solver::DynamicsConstraint> _dynamicsConstraint;

    // BSpline curve
    std::shared_ptr<mc_tasks::BSplineTrajectoryTask> BSplineVel;
    std::shared_ptr<mc_tasks::VectorOrientationTask> _vectorOrientationTask;

    sva::PTransformd _initial_hammerhead_position;
    sva::PTransformd _initial_nail_position;    
    mc_trajectory::BSpline::waypoints_t _posWp = {};
    std::vector<std::pair<double, Eigen::Matrix3d>> _oriWp = {};

    typedef Eigen::Vector3d Point;
    typedef Point point_t;
    typedef ndcurves::curve_constraints<point_t> curve_constraints_t;
    sva::PTransformd _target;
    
    typedef Eigen::Vector3d vector3_t;
    vector3_t _start_point;
    vector3_t _end_point;
    bool stop = false;
    double _run_exec_time = 0.002;
    //Position task test
    std::shared_ptr<mc_tasks::PositionTask> _positionTask;

    // ROS
//     rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr _subForce;


    // Forces
    vector3_t _force_vector;
    double _max_force_norm = 0;
    double _max_normal_force_norm = 0;

    vector3_t _peak_force = {0, 0, 0};
    vector3_t _peak_normal_force = {0, 0, 0};
    Eigen::Matrix<double, 3, 1> _rotation_axis;
    

    bool _impact_detected = false;
  
    /**
    @brief Do not use that function
    Uses the brute force algorithm from my internship report (bad way to do it)
    */
    const double compute_effective_mass_naive(rbd::MultiBodyConfig mbc, 
                                              mc_control::fsm::Controller & ctl_) const;
    
    /**
    @brief Compute the effective mass using the mbc object (rbd::MultiBodyConfiguration)
    @param mbc the MultiBodyConfig of the robot
    @param ctl_
    @param normal_vector the vector used to compute the effective mass of the robot
    */
    const double compute_effective_mass_with_mbc(rbd::MultiBodyConfig mbc, 
                                                mc_control::fsm::Controller & ctl_, 
                                                const Eigen::Vector3d &normal_vector) const;


    /**
    @brief Compute the effective mass using the encoders
    @param encoderValues the encoder values of the robot
    @param ctl_
    @param normal_vector the vector used to compute the effective mass of the robot
     */
    const double compute_effective_mass_with_encoders(const std::vector<double> &encoderValues, 
                                                      mc_control::fsm::Controller & ctl_, 
                                                      const Eigen::Vector3d &normal_vector) const;

    /**
    @brief Compute the derivative of the effective mass with respect to the robot configuration
          using central difference
    @param mbc the MultiBodyConfig of the robot
    @param ctl_
    @param normal_vector the vector used to compute the effective mass of the robot
     */                                                 
    const Eigen::VectorXd compute_emass_gradient_central_difference_mbc(const rbd::MultiBodyConfig &mbc, 
                                                                        mc_control::fsm::Controller &ctl_, 
                                                                        const Eigen::Vector3d &normal_vector) const;

    /**
    @brief Compute the derivative of the effective mass with respect to the robot configuration
          using backward difference
    @param mbc the MultiBodyConfig of the robot
    @param ctl_
    @param normal_vector the vector used to compute the effective mass of the robot
     */  
    const Eigen::VectorXd compute_emass_gradient_backward_difference_mbc(const rbd::MultiBodyConfig &mbc, 
                                                                          mc_control::fsm::Controller &ctl_, 
                                                                          const Eigen::Vector3d &normal_vector) const;

    /**
    @brief Compute the derivative of the effective mass with respect to the robot configuration
          using 3-point backward difference
    @param mbc the MultiBodyConfig of the robot
    @param ctl_
    @param normal_vector the vector used to compute the effective mass of the robot
    */ 
    const Eigen::VectorXd compute_emass_gradient_three_point_backward_difference_mbc(const rbd::MultiBodyConfig &mbc, 
                                                                          mc_control::fsm::Controller &ctl_, 
                                                                          const Eigen::Vector3d &normal_vector) const;

    /**
    @brief Compute the derivative of the effective mass with respect to the robot configuration
            using central difference and the encoder values
    @param encoderValues the encoder values of the robot
    @param ctl_
    @param normal_vector the vector used to compute the effective mass of the robot
    */
    const Eigen::VectorXd compute_emass_gradient_central_difference_encoders(const std::vector<double> &encoderValues, 
                                                                              mc_control::fsm::Controller &ctl_, 
                                                                              const Eigen::Vector3d &normal_vector) const;
                                                                              
    /**
    @brief Outputs a csv file to visualize an Eigen matrix
    @param matrix the matrix to visualize
    @param filemame the name of the output file
    */ 
    void writeEigenMatrixToCSV(const Eigen::MatrixXd& matrix, const std::string& filename) const;

    /**
    @brief Outputs a csv file to visualize an std::vector
    @param vector the vector to visualize
    @param filemame the name of the output file
    */ 
    void writeVectorToCSV(const std::vector<double> & vec, const std::string & filename) const;

    /**
    @brief Prints the configuration q given a MultiBodyConfig object
    @param mbc the MultiBodyConfig of the robot
    @param string some string like "q = "
    */ 
    void printConfig(rbd::MultiBodyConfig mbc, std::string string) const;
    
    int _nrdof = 41;
    double _effective_mass = 0;
    bool _create_file = true;
    std::vector<double> _masses;

    //finite differences dm/dt
    rbd::MultiBodyConfig _old_mbc;
    rbd::MultiBodyConfig _new_mbc;
    double _old_effective_mass_mbc = 1;
    double _old_floating_base_effective_mass_mbc = 1;
    double _old_reduced_effective_mass_mbc = 1;
    std::vector<double> _ratios;
    double _old_effective_mass_encoders = 1;

    const Eigen::Vector3d _normal_vector_nail_frame = {0, 0, 1};
    Eigen::Vector3d _normal_vector_world_frame = {0, 0, 0};

    /**
    @brief Computes the time derivative of the effective mass using its gradient and mbc.q
    @param gradient the gradient of the effective mass of the robot
    @param ctl_ to access mbc.q
    */
    const double emass_time_derivative_with_mbc_q_derivative(const Eigen::VectorXd &gradient, 
                                                        mc_control::fsm::Controller & ctl_) const;

    /**
    @brief Computes the time derivative of the effective mass using its gradient and the encoder values of the robot
    @param gradient the gradient of the effective mass of the robot
    @param ctl_ to access the encoder values of the robot
    */
    const double emass_time_derivative_with_encoders(const Eigen::VectorXd &gradient, 
                                                    mc_control::fsm::Controller & ctl_) const;

    /**
    @brief Do not use that function.
    Computes the time derivative of the effective mass using its gradient and mbc.alpha
    @param gradient the gradient of the effective mass of the robot
    @param ctl_ to access mbc.alpha
    */
    const double emass_time_derivative_with_mbc_alpha(const Eigen::VectorXd &gradient, 
                                                    mc_control::fsm::Controller & ctl_) const;


    /**
    @brief Estimates the effective mass at the previous iteration using the gradient and the current effective mass
    @param gradient the gradient of the effective mass of the robot
    @param current_m the current effective mass of the robot
    */
    const double estimate_previous_effective_mass(const Eigen::VectorXd &gradient, const double &current_m) const;

    /**
    @brief Displays in the terminal the robot configuration obtained using various ways
    @param gradient the gradient of the effective mass of the robot
    @param current_m the current effective mass of the robot
    */
    void compare_configs(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_);
    std::map<std::string, double> _old_q_mbc_map;
    std::map<std::string, double> _old_q_encoders_map;
    double _old_floating_base_effective_mass = 0;
    std::vector<double> _old_q_encoders = {};
    double _integrated_effective_mass_encoders = 0;
    double _integrated_effective_mass_mbc = 0;
    rbd::MultiBodyConfig _integrated_mbc;

    bool _first_iteration = true;
    rbd::MultiBodyConfig _initial_mbc;

    /**
    @brief Reorganize the order of the gradient of the effective mass computed with encoder values to match the mb object
          because the order of the joints using encoders is not the same as the order of the mb object
    @param gradient the gradient of the effective mass to reorganize
    */
    const Eigen::VectorXd reorganized_gradient(const Eigen::VectorXd &gradient,
                                              mc_control::fsm::Controller & ctl_) const;

    /**
    @brief Convert the encoder values to mbc
    @param encoderValues the encoder values of the robot
    @param ctl_
    */
    const rbd::MultiBodyConfig encoders_values_to_mbc(const std::vector<double> &encoderValues, mc_control::fsm::Controller & ctl_) const;

    /**
    @brief Convert the encoder velocities to mbc
    @param encoderValues the encoder velocities of the robot
    @param ctl_
    */
    const rbd::MultiBodyConfig encoders_velocities_to_mbc(const std::vector<double> &encoderVelocities,
                                                                        mc_control::fsm::Controller & ctl_) const;
    

    /**
    @brief Reconstructs the velocity of the bezier curve given a shared_ptr of a BSplineTrajectoryTask
           by time differentiating with central difference
          @param BSpline the shared_ptr of a BSplineTrajectoryTask
          @param ctl_
     */            
    const Eigen::Vector3d bezier_vel_from_task(const std::shared_ptr<mc_tasks::BSplineTrajectoryTask> &BSplineVel,
                                              mc_control::fsm::Controller & ctl_) const;
                                              
    /**
    @brief Comptues the projected momentum onto the normal vector 
            given a velocity vector and an effective mass 
    @param velocity_vector the 3d velocity vector
    @param effective_mass the effective mass of the robot
    @param normal_vector the normal vector used to compute the effective mass of the robot
     */
    const double compute_projected_momentum(const double &effective_mass,
                                  const Eigen::Vector3d &velocity_vector, 
                                  const Eigen::Vector3d &normal_vector) const;                                                           
    double _total_time_elapsed = 0.0f;

    const Eigen::Matrix3d roll_rotation_nail_frame(const double &angle) const;
    const Eigen::Matrix3d pitch_rotation_nail_frame(const double &angle) const;
    const Eigen::Matrix3d yaw_rotation_nail_frame(const double &angle) const;

    // -------------------------------- Parameters ---------------------------------------
    
    // Parameters loaded in the load_parameters function, parameters are found in the Hammering_FSM_Controller.in.yaml file
    // Don't ask me why there is a '.in' in the name of the file, I don't know 
    
    mc_rtc::Configuration _config;

    /**
    @brief Loads the parameters found in the Hammering_FSM_Controller.in.yaml file
     */
    void load_parameters();
    
    std::string _nail_robot_name = "nail";
    std::string _main_robot_name = "hrp5_p";
    std::string _hammer_head_frame_name = "Hammer_Head";
    std::string _nail_frame_name = "nail";
    
    // timestep
    double _timestep = 1;

    // quality of life
    
    bool _bezier_curve_verbose_active = false;
    bool _jacobian_verbose_active = false;

    
    // gui
    
    std::string _stop_hammering_button_name = "undefined";
    
    //curve constraints of the bezier curve
    
    curve_constraints_t _constr;
    
    // Default magic values, just for testing, all loaded in the load_parameters function
    
    double _magic_max_control_point_height = 1;
    double _magic_bezier_curve_max_duration = 1;
    double _magic_task_stiffness = 1;
    double _magic_task_weight = 1;
    double _magic_epsilon = 1;
    double _magic_oriWp_time = 1;
    double _posture_task_weight = 1;
    double _effective_mass_maximization_task_weight = 1;

    
    double _magic_epsilon_force_norm_threshold = 1;

};
