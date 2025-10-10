// Written by Jimmy VU

#pragma once
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
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
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "rclcpp/rclcpp.hpp" //including ros2
#include <mc_rtc_ros/ros.h>

#include <RBDyn/Jacobian.h>
#include <vector>

#include <mc_tasks/VectorOrientationTask.h>
typedef Eigen::Vector3d Point;
typedef Point point_t;
typedef ndcurves::curve_constraints<point_t> curve_constraints_t;
// typedef Eigen::Vector3d vector3_t;

struct Get_In_Position_Task : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  private:

    // BSpline curve
    std::shared_ptr<mc_tasks::BSplineTrajectoryTask> _BSplineVel;
    std::shared_ptr<mc_tasks::VectorOrientationTask> _vectorOrientationTask;
    mc_trajectory::BSpline::waypoints_t _posWp = {};
    std::vector<std::pair<double, Eigen::Matrix3d>> _oriWp = {};

    //curve constraints of the bezier curve
    curve_constraints_t _constr;

    // Nail target
    sva::PTransformd _target;
    Eigen::Vector3d _end_point;

    bool stop = false;
    
    // Size of the gradient of m is [1 x n_dof]
    Eigen::VectorXd _gradient_of_m;
    
    double _total_time_elapsed = 0.0f;
    
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
    /**
    @brief Compute the angle error between two 3D vectors using the dot product
     */
    const double vector_error(const Eigen::Vector3d &va, const Eigen::Vector3d &vb) const;



    // ------------------------------------- Parameters -------------------------------
    mc_rtc::Configuration _config;

    void load_params();

    double _magic_BSpline_max_duration = 1.0f;
    double _magic_BSpline_task_stiffness = 1.0f; 
    double _magic_BSpline_task_weight = 1.0f;

    //V^w_f,in, c.f. article    
    double _magic_normal_final_velocity = 1.0f;

    //W_p, c.f. article or internship report
    double _magic_posture_task_weight = 1.0f;

    //W_m, c.f. article or internship report
    double _magic_effective_mass_maximization_task_weight = 1.0f;


    double _magic_vector_orientation_task_dimweight_x = 1.0f;
    double _magic_vector_orientation_task_dimweight_y = 1.0f;
    double _magic_vector_orientation_task_dimweight_z = 1.0f;
    double _magic_vector_orientation_task_weight = 1.0f;
    double _magic_vector_orientation_task_stiffness = 1.0f;



    bool _enable_BSpline_orientation = false;





    // --------------------------- Not needed to run in the code but usable for debugging ----------------------------------

    const Eigen::Matrix3d roll_rotation_nail_frame(const double &angle) const;
    const Eigen::Matrix3d pitch_rotation_nail_frame(const double &angle) const;
    const Eigen::Matrix3d yaw_rotation_nail_frame(const double &angle) const;

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
    @brief Do not use that function
    Uses the brute force algorithm from my internship report (bad way to do it)
    */
    const double compute_effective_mass_naive(rbd::MultiBodyConfig mbc, 
                                              mc_control::fsm::Controller & ctl_) const;

    bool _create_file = true;

    // Quaternion rotation axis
    Eigen::Matrix<double, 3, 1> _rotation_axis;

    //finite differences dm/dt
    rbd::MultiBodyConfig _old_mbc;
    rbd::MultiBodyConfig _new_mbc;
    double _old_effective_mass_mbc = 1;
    double _old_floating_base_effective_mass_mbc = 1;
    double _old_reduced_effective_mass_mbc = 1;
    std::vector<double> _ratios;
    double _old_effective_mass_encoders = 1;    

};
