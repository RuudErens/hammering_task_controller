#include "Get_In_Position_Task.h"

#include "../HammeringTaskNew.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <RBDyn/Jacobian.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <SpaceVecAlg/EigenTypedef.h>
#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <map>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/PositionTask.h>
#include <mc_tasks/VectorOrientationTask.h>
#include <memory>
#include <ndcurves/bezier_curve.h>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>


// #include <pinocchio/parsers/urdf.hpp>


void Get_In_Position_Task::configure(const mc_rtc::Configuration & config)
{
   _config.load(config);
    // mc_rtc::log::info("Get_In_Position_Task configure function called with config : \n{}", config.dump(true, true));
    
}

void Get_In_Position_Task::start(mc_control::fsm::Controller & ctl_)
{
  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);

  load_parameters();
  _dynamicsConstraint = std::make_unique<mc_solver::DynamicsConstraint>(ctl.robots(), 
                                                                        ctl.robot().robotIndex(),
                                                                      ctl.solver().dt(),
                                                                      std::array<double, 3>{0.1, 0.01, 0.5},
                                                                      1.0,
                                                                    true);
  mc_rtc::log::info("solver timestep = {} s", ctl.solver().dt());
  _nrdof = ctl.robot().mb().nrDof();
 
  // for (size_t i = 0; i < std::size(ctl.robot().mbc().q); ++i)
  // {
  //   std::cout << ctl_.robot().mb().joint(i).name()<< " : " ; 
  //   std::cout << "{";
  //   for(size_t j = 0; j < std::size(ctl.robot().mbc().q.at(i)); ++j)
  //   { 
  //     std::cout << ctl.robot().mbc().q.at(i).at(j) << ", ";
  //     // n+=1;
  //     // Q.push_back(qf.q.at(i).at(j));
  //   }
  //   std::cout << "}" << std::endl;
  // }
  // std::cout << "}" << std::endl;
  // std::cout << "robot pos world = \n" << ctl.robot().posW() << std::endl;
  // Add a stop button to the gui
  ctl.gui()->addElement({}, mc_rtc::gui::Button(_stop_hammering_button_name, [this]() { stop = true; }));
  
  // Get the initial conditions
  // for (const auto &frame : ctl.robot().frames())
  // {
  //   std::cout << frame << "\n";
  // }
  // std::cout << "\n";
  // std::cout << ctl.robot().frame(_hammer_head_frame_name).body() << std::endl;
  // std::cout << ctl.robot().frame("LeftHand").position().rotation() << std::endl;

  // std::cout << "base frame rotation matrix = \n" << ctl.robot().frame("base_link").position().rotation() << "\n";
  // printConfig(ctl.robot().mbc(), "q_init");

  _start_point = ctl.robot().frame(_hammer_head_frame_name).position().translation();
  _end_point = ctl.robots().robot(_nail_robot_name).frame(_nail_frame_name).position().translation();

  double normal_final_velocity = -2;

  // Found by calculations by hand
  _rotation_axis = Eigen::Matrix<double, 3, 1>(1,0,-1).normalized();
  auto angle = M_PI;
  // The quaternion works for a nail placed on a horizontal table, it will not work for a nail placed on a slope
  // The angle and the rotation axis should be computed for different orientations of the nail, but I did not do it
  Eigen::Quaterniond q(Eigen::AngleAxisd(angle, _rotation_axis));


  // I dont need to specify the endpoint as a posWp because _target takes care of that
  // If I do specify it, then it would add 1 degree to the curve even though the points are the same
  // _posWp ={_end_point};
          
  // Eigen::Matrix3d pi_z_rot;
  // pi_z_rot << cos(M_PI), sin(M_PI), 0,
  //             -sin(M_PI), cos(M_PI), 0,
  //             0, 0, 1;

  // Eigen::Matrix3d pi_y_rot;
  // pi_y_rot << cos(-M_PI/2),  0, sin(-M_PI/2),
  //                   0,          1,        0,
  //             -sin(-M_PI/2), 0, cos(-M_PI/2);
  // Eigen::Matrix3d pi_x_rot;
  // pi_x_rot << 1,      0,               0,
  //             0, cos(-M_PI/2), -sin(-M_PI/2),
  //             0, sin(-M_PI/2),  cos(-M_PI/2);

  Eigen::Matrix3d nail_rot = ctl.robot("nail").frame("nail").position().rotation();
  // Found by using Rviz by first taking the nail rotation matrix and then making rotations to reach the correct result
  Eigen::Matrix3d additional_rot = 
  roll_rotation_nail_frame(-M_PI/2)*
                                    pitch_rotation_nail_frame(-M_PI_2)*
                                    yaw_rotation_nail_frame(M_PI);
  Eigen::Matrix3d needed_hammer_rot = additional_rot*nail_rot;
  Eigen::Matrix3d id = Eigen::Matrix3d::Identity();
  Eigen::Quaterniond nail_quaternion(id);
  // Eigen::Quaterniond nail_quaternion(id);

  _normal_vector_world_frame = (nail_rot.transpose()*_normal_vector_nail_frame).normalized();
  _constr.end_vel.x() = normal_final_velocity*_normal_vector_world_frame.x();
  _constr.end_vel.y() = normal_final_velocity*_normal_vector_world_frame.y();
  _constr.end_vel.z() = normal_final_velocity*_normal_vector_world_frame.z();
  _oriWp = {
    std::make_pair(_magic_oriWp_time, nail_quaternion.matrix()) // at t = _magic_oriWp_time, arbitrary for now
  };
  // _normal_vector_world_frame = nail_rot.transpose()*_normal_vector_world_frame ;
  
  _target = sva::PTransformd(nail_quaternion, //maybe the quaternion expresses the orientation of your rotating frame you want to achieve at the end with respect to the world frame
                                                            //thus whatever the starting orientation of the rotating frame wrt the world frame, the robot frame will try to end up at the orientation specified by the quaternion
                                                            //how does it do that ?
                            _end_point
                            );
  
  // The curve has at least 2 control points : the first one being the initial translation of the frame and the second 
  // being the target, thus the curve is at least of degree 1
  BSplineVel = std::make_shared<mc_tasks::BSplineTrajectoryTask>(ctl.robot().frame(_hammer_head_frame_name),
                                                                  _magic_bezier_curve_max_duration,
                                                                  _magic_task_stiffness, 
                                                                  _magic_task_weight, 
                                                                  _target, 
                                                                  _constr,
                                                                  _posWp, 
                                                                  _oriWp);
                                                                  // _bezier_curve_verbose_active);
  Eigen::Vector3d normal_vector_to_align_in_hammerhead_frame(1, 0,0); 
  _vectorOrientationTask = std::make_shared<mc_tasks::VectorOrientationTask>(ctl.robot().frame(_hammer_head_frame_name),
                                                                            normal_vector_to_align_in_hammerhead_frame                                                       
  );
  _vectorOrientationTask->targetVector(-_normal_vector_world_frame);
  _vectorOrientationTask->weight(100);
  _vectorOrientationTask->stiffness(250);
  ctl.solver().addTask(_vectorOrientationTask);
  BSplineVel->stiffness(_magic_task_stiffness);
  BSplineVel->weight(_magic_task_weight);

  Eigen::Vector6d dimweights = BSplineVel->dimWeight();
  // Remove the orientation part of the BSpline by setting the orientation weights to 0
  dimweights(0) = 0;
  dimweights(1) = 0;
  dimweights(2) = 0;
  // Increase the weights on the x and y coordinates
  dimweights(3) = 10000;
  dimweights(4) = 10000;
  BSplineVel->dimWeight(dimweights);

  mc_rtc::log::info("Degree of BSpline : {}", BSplineVel->spline().get_bezier()->degree());
  // ctl.getPostureTask(ctl.robot().name())->weight(_posture_task_weight);
  ctl.solver().addTask(BSplineVel);
  // _positionTask = std::make_shared<mc_tasks::PositionTask>(ctl.robot().frame(_hammer_head_frame_name), 2.0, 10);
  // _positionTask->position(_end_point);
  // ctl.solver().addTask(_positionTask);
  ctl.solver().addConstraintSet(_dynamicsConstraint);
  // std::cout << "torque = {" << std::endl;
  // for(const auto &joint : ctl.robot().mbc().jointTorque)
  // {
  //   std::cout << "{";
  //   for(const double &torque : joint)
  //   {
  //     std::cout << torque << ", ";
  //   }
  //   std::cout << "}" << std::endl;
  // }
  // std::cout << "}" << std::endl;
  _old_q_encoders = ctl_.robot().encoderValues();
  _initial_mbc = ctl.robot().mbc();
  _integrated_mbc = _initial_mbc;

}


bool Get_In_Position_Task::run(mc_control::fsm::Controller & ctl_)
{
  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);
  _new_mbc = ctl.robot().mbc();
  // mc_rtc::log::info("normal vector world frame = {} ", _normal_vector_world_frame);
  // mc_rtc::log::info("dimweight = {} ", BSplineVel->dimWeight());

  if(_first_iteration)
  {
    // For some reason the mass matrix is null at the very first iteration, Thomas said it was a bug getting fixed


    //Initial conditions
    const double initial_effective_mass_encoders  = compute_effective_mass_with_encoders(_old_q_encoders, ctl, _normal_vector_world_frame);
    const double initial_effective_mass_mbc  = compute_effective_mass_with_mbc(_initial_mbc, ctl, _normal_vector_world_frame);

    _integrated_effective_mass_encoders = initial_effective_mass_encoders;
    _integrated_effective_mass_mbc = initial_effective_mass_mbc;

    _old_mbc = ctl.robot().mbc();
    _old_effective_mass_mbc = compute_effective_mass_with_mbc(_old_mbc, ctl, _normal_vector_world_frame);
    _old_effective_mass_encoders = compute_effective_mass_with_encoders(ctl.robot().encoderValues(), ctl, _normal_vector_world_frame);
    _first_iteration = !_first_iteration;
    // dqi is 0 during the first iteration


    // Add some entries to the logs
    ctl.logger().addLogEntry("Effective mass [kg]", this, [&, this]()
    {return compute_effective_mass_with_mbc(_new_mbc, ctl, _normal_vector_world_frame);});

      ctl.logger().addLogEntry("Hammer tip velocity [m/s]", this, [&, this]()
    {return ctl.robot().frame(_hammer_head_frame_name).velocity().linear();});
      
    ctl.logger().addLogEntry("Hammer tip reference bezier velocity [m/s]", this, [&, this]()
    {return bezier_vel_from_task(BSplineVel, ctl);});
    
    ctl.logger().addLogEntry("Projected momentum of hammer tip [kgm/s]", this, [&, this]()
    {
      Eigen::Vector3d velocity_vector = bezier_vel_from_task(BSplineVel, ctl);
      double effective_mass = compute_effective_mass_with_mbc(_new_mbc, ctl, _normal_vector_world_frame);
      return compute_projected_momentum(effective_mass, velocity_vector, _normal_vector_world_frame);
    });

    _total_time_elapsed+=ctl.solver().dt();
    return false;
  }
  // Eigen::VectorXd gradient_of_m = compute_emass_gradient_central_difference_mbc(_new_mbc, 
  //                                                                             ctl, 
  //                                                                   _normal_vector_world_frame);
  Eigen::VectorXd gradient_of_m = compute_emass_gradient_three_point_backward_difference_mbc(_new_mbc, 
                                                                            ctl, 
                                                                _normal_vector_world_frame);
  Eigen::VectorXd gradient_of_m_three_point = compute_emass_gradient_three_point_backward_difference_mbc(_new_mbc, 
                                                                            ctl, 
                                                                _normal_vector_world_frame);                                                              
  // mc_rtc::log::info("Gradient of m = {}", gradient_of_m);
  // mc_rtc::log::info("Gradient of m three point = {}", gradient_of_m_three_point);

  // mc_rtc::log::info("Degree of BSpline : {}", BSplineVel->spline().get_bezier()->degree());
  // mc_rtc::log::info("Number of control points : {}", std::size(BSplineVel->spline().get_bezier()->waypoints()));

  // for(size_t i = 0; i < std::size(BSplineVel->spline().get_bezier()->waypoints()); ++i)
  // {
  //   mc_rtc::log::info("control point[{}] : {}", i, BSplineVel->spline().get_bezier()->waypoints().at(i));
  // }

  // mc_rtc::log::info("q used = ");
  unsigned int k = 0;
  for(size_t i = 0; i < std::size(_new_mbc.q); ++i)
  {
    if(std::size(_new_mbc.q.at(i)) != 1)
    {
        // mc_rtc::log::info("({}) []", ctl.robot().mb().joint(i).name());
        continue;
    }
    double dqi_mbc_backward = _new_mbc.q.at(i).at(0) - _old_mbc.q.at(i).at(0);
    double dqi_encoders_backward = encoders_values_to_mbc(ctl.robot().encoderValues(), ctl_).q.at(i).at(0) 
                                  - encoders_values_to_mbc(_old_q_encoders, ctl_).q.at(i).at(0);
    double qi_dot_mbc_backward = dqi_mbc_backward/ctl.solver().dt(); 
    double qi_dot_encoders_backward = dqi_mbc_backward/ctl.solver().dt(); 

    // mc_rtc::log::info("({}) [q = {}], [q_dot_mbc = {}] [q_dot_encoders = {}], grad[{}] = {}",ctl.robot().mb().joint(i).name(), 
    //                                                       _new_mbc.q.at(i).at(0),
    //                                                       qi_dot_mbc_backward, 
    //                                                        qi_dot_encoders_backward,
    //                                                         k, 
    //                                                         gradient_of_m(k, 0));

    
    // mc_rtc::log::info("({}) grad[{}] = {}",ctl.robot().mb().joint(i).name(), 
    //                                                         k, 
    //                                                         gradient_of_m(k, 0));


    // mc_rtc::log::info("({}) [q = {}], [dq_mbc = {}] [dq_encoders = {}]",ctl.robot().mb().joint(i).name(), 
    //                                                       _new_mbc.q.at(i).at(0),
    //                                                       qi_dot_mbc_backward*ctl.solver().dt(), 
    //                                                        qi_dot_encoders_backward*ctl.solver().dt());
    _integrated_mbc.q.at(i).at(0) += qi_dot_mbc_backward*ctl.solver().dt();                                
    // mc_rtc::log::info("({}) [q_mbc = {}], [q_predicted_mbc = {}]",
    //                   ctl.robot().mb().joint(i).name(), 
    //                    _new_mbc.q.at(i).at(0),
    //                   _integrated_mbc.q.at(i).at(0));                                                        
    ++k;
  } 
  double effective_mass_mbc = compute_effective_mass_with_mbc(_new_mbc, ctl, _normal_vector_world_frame);
  double effective_mass_encoders = compute_effective_mass_with_encoders(ctl_.robot().encoderValues(), 
                                                                ctl, 
                                                                _normal_vector_world_frame);
  double dm_mbc_backward = effective_mass_mbc - _old_effective_mass_mbc;
  double dmdt_mbc = dm_mbc_backward / ctl.solver().dt();
  double dmdt_encoders = emass_time_derivative_with_encoders(gradient_of_m, ctl);
  _integrated_effective_mass_mbc += dmdt_mbc * ctl.solver().dt();
  _integrated_effective_mass_encoders += dmdt_encoders * ctl.solver().dt();

  // mc_rtc::log::info("integrated effective mass encoders = {} kg", _integrated_effective_mass_encoders);
  // mc_rtc::log::info("integrated effective mass mbc = {} kg", _integrated_effective_mass_mbc);

  // return true;
  // Eigen::VectorXd gradient_of_m_encoders = compute_emass_gradient_central_difference_encoders(ctl_.robot().encoderValues(), 
  //                                                                                                       ctl, 
  //                                                                                             _normal_vector_world_frame);

    // compare_configs(gradient_of_m, ctl_);

  //   mc_rtc::log::info("old_effective_mass_mbc = {} kg", _old_effective_mass_mbc);
    // mc_rtc::log::info("Effective mass mbc of the robot = {} kg", effective_mass_mbc);
    // mc_rtc::log::info("Effective mass encoders of the robot = {} kg", effective_mass_encoders);
  
    // _old_effective_mass_mbc = compute_effective_mass_with_mbc(_old_mbc, 
    //                                                               ctl_, 
    //                                                   _normal_vector_world_frame);
    double dm_encoders_backward = effective_mass_encoders - _old_effective_mass_encoders;
    double dm_encoders_product = dmdt_encoders * ctl.solver().dt();
    // mc_rtc::log::info("dm/dt mbc by finite difference = {} kg/s", dmdt_mbc);
  //   mc_rtc::log::info("Effective mass encoders of the robot = {} kg", effective_mass_encoders);
    // mc_rtc::log::info("dm/dt encoders by finite difference = {} kg/s", dmdt_encoders);
    double dmdt_mbc_with_dq = emass_time_derivative_with_mbc_q_derivative(gradient_of_m, ctl_);
    double dm_mbc_with_dq = dmdt_mbc_with_dq * ctl.solver().dt();
    // mc_rtc::log::info("dm/dt mbc by product with q_mbc_derivative = {} kg/s", dmdt_mbc_with_dq);
    // mc_rtc::log::info("dm_mbc_backward = {} kg", dm_mbc_backward);
    // mc_rtc::log::info("dm_mbc_with_dq = {} kg", dm_mbc_with_dq);
    // mc_rtc::log::info("dm_encoders_with_qdot = {} kg", dm_encoders_product);
    double estimated_previous_effective_mass = estimate_previous_effective_mass(gradient_of_m, effective_mass_mbc);
    double estimated_dm_with_grad = effective_mass_mbc - estimated_previous_effective_mass;
    double dmgrad_dm_ratio = dm_mbc_with_dq/dm_mbc_backward;
    double dmgrad_dm_ratio_2 = estimated_dm_with_grad/dm_mbc_backward;
    // mc_rtc::log::info("dmgrad/dm ratio = {}", dmgrad_dm_ratio);
    // mc_rtc::log::info("dmgrad_dm_ratio_2 = {}", dmgrad_dm_ratio_2);

    // double dm_error = std::abs(dm_mbc_backward - estimated_dm_with_grad);
    // if(std::abs(estimated_dm_with_grad) > std::abs(dm_mbc_backward))
    // {
    //   mc_rtc::log::info("estimated_dm_with_grad is bigger = {} kg", std::abs(estimated_dm_with_grad));
    // }
    // else {
    //   // mc_rtc::log::info("dm_mbc_backward is bigger = {} kg", std::abs(dm_mbc_backward));
    // }
    // double error = dm_error / std::max({std::abs(dm_mbc_backward), std::abs(estimated_dm_with_grad)});
    // mc_rtc::log::info("dm_error = {} kg", dm_error);
    // mc_rtc::log::info("error = {}%", 100*error);
    // mc_rtc::log::info("dm/dt encoders by product with q_dot_encoders = {} kg/s", emass_time_derivative_with_encoders(gradient_of_m, ctl_));
    // mc_rtc::log::info("old previous mass = {} kg", _old_effective_mass_mbc);
    
    // mc_rtc::log::info("estimated_previous_effective_mass = {} kg", estimated_previous_effective_mass);
    // mc_rtc::log::info("r = {}%", 100*estimated_previous_effective_mass/_old_effective_mass_mbc);
    // mc_rtc::log::info("-------------------------");

    // _ratios.push_back(_old_effective_mass_encoders/estimated_previous_effective_mass);
  //   mc_rtc::log::info("dm/dt mbc by product with mbc.alpha = {} kg/s", emass_time_derivative_with_mbc_alpha(gradient_of_m, ctl_));
    _old_mbc = _new_mbc;
    _old_effective_mass_mbc = effective_mass_mbc;
    _old_effective_mass_encoders = effective_mass_encoders;
    _old_q_encoders = ctl.robot().encoderValues();

    

    // mc_rtc::log::info("posture task ref accel = {}", ctl.getPostureTask(ctl.robot().name())->refAccel());
    ctl.getPostureTask(ctl.robot().name())->refAccel((_effective_mass_maximization_task_weight/(_posture_task_weight)) * gradient_of_m);
    
    // mc_rtc::log::info("Posture objective = {");
    // for (size_t i = 0; i < std::size(ctl.getPostureTask(ctl.robot().name())->posture()); ++i)
    // {
    //   for(size_t j = 0; j < std::size(ctl.getPostureTask(ctl.robot().name())->posture().at(i)); ++j)
    //   {
    //     mc_rtc::log::info("[{}]", ctl.getPostureTask(ctl.robot().name())->posture().at(i).at(j));
    //   }
    // }
    // mc_rtc::log::info("}");
    // mc_rtc::log::info("Current Posture = {");
    // for (size_t i = 0; i < std::size(ctl.robot().mbc().q); ++i)
    // {
    //   for(size_t j = 0; j < std::size(ctl.robot().mbc().q.at(i)); ++j)
    //   {
    //     mc_rtc::log::info("[{}]", ctl.robot().mbc().q.at(i).at(j));
    //   }
    // }
    // mc_rtc::log::info("}");
    // mc_rtc::log::info("Posture refVel = {}", ctl.getPostureTask(ctl_.robot().name())->);
    // mc_rtc::log::info("Posture refVel = {}", ctl.getPostureTask(ctl_.robot().name())->refVel());
    // mc_rtc::log::info("Posture refAcc = {}", ctl.getPostureTask(ctl_.robot().name())->refAccel());
    // mc_rtc::log::info("Posture stiffness = {}", ctl.getPostureTask(ctl_.robot().name())->stiffness());
    // mc_rtc::log::info("Posture damping = {}", ctl.getPostureTask(ctl_.robot().name())->damping());
// ctl.getPostureTask(ctl_.robot().name())->
    // mc_rtc::log::info("Gradient of m = \n {}", gradient_of_m);
    // mc_rtc::log::info("Gradient of m_encoders = \n {}", gradient_of_m_encoders);
    
    // auto tasks = ctl.solver().tasks();
    // for(const auto & task : tasks)
    // {
    //   mc_rtc::log::info("Task {}", task->name());
    // }
    // BSplineVel->weight(10/BSplineVel->eval().norm());
    // return true;
    // mc_rtc::log::info("eval norm = {} ", BSplineVel->eval().norm());
    // ctl.getPostureTask(ctl.robot().name())->refAccel((_effective_mass_maximization_task_weight/(_posture_task_weight*_posture_task_weight)) * gradient_of_m);
    // mc_rtc::log::info("---------------------------");

    // if (_create_file)
    // {
    //   mc_rtc::log::info("Effective mass of the robot at the nail = {} kg", compute_effective_mass_with_mbc(ctl.robot().mbc(), ctl_, _normal_vector_world_frame));
    //   _create_file = false;
    // }
    
    if( BSplineVel->eval().norm() < _magic_epsilon)
    {
      // _effective_mass = compute_effective_mass_naive(ctl.robot().mbc(), ctl_);
      // writeVectorToCSV(_masses, "Masses_vector.csv");
      // mc_rtc::log::info("Effective mass = {} kg", _effective_mass);
      output("STOP");
      return true;
    }
  _total_time_elapsed+=ctl.solver().dt();
  return false;
}

void Get_In_Position_Task::teardown(mc_control::fsm::Controller & ctl_)
{
  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);

  ctl_.gui()->removeElement({}, _stop_hammering_button_name);

  ctl.solver().removeTask(BSplineVel);
  ctl.getPostureTask(ctl.robot().name())->weight(10);
}

const Eigen::Vector3d Get_In_Position_Task::bezier_vel_from_task(
  const std::shared_ptr<mc_tasks::BSplineTrajectoryTask> &BSplineVel,
  mc_control::fsm::Controller & ctl_) const 
{
  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);
  ndcurves::bezier_curve bezier_curve = *BSplineVel->spline().get_bezier();
  Eigen::Vector3d p_past = bezier_curve(_total_time_elapsed - ctl.solver().dt());
  Eigen::Vector3d p_future = bezier_curve(_total_time_elapsed + ctl.solver().dt());

  return (p_future - p_past)/(2*ctl.solver().dt());
}                                                                  



const double Get_In_Position_Task::compute_projected_momentum(
  const double &effective_mass,
  const Eigen::Vector3d &velocity_vector, 
  const Eigen::Vector3d &normal_vector) const
{
  return effective_mass* (velocity_vector.x()*normal_vector.x()
                          + velocity_vector.y()*normal_vector.y()
                          + velocity_vector.z()*normal_vector.z());
}                                                          

const double Get_In_Position_Task::estimate_previous_effective_mass(const Eigen::VectorXd &gradient, const double &current_m) const{
  double dm_sum = 0;
  double dqi = 0;
  std::vector<std::vector<double>> q = _new_mbc.q;
  std::vector<std::vector<double>> old_q = _old_mbc.q;
  unsigned int j = 0;
  for (size_t i = 0; i < std::size(q); ++i)
  {
    if (std::size(q.at(i)) != 1)
    {
      //Ignore floating base (size = 6) and empty dofs (size = 0)
      continue;
    }    
    dqi = q.at(i).at(0) - old_q.at(i).at(0);
    dm_sum += gradient(j, 0) * dqi;
    j+=1;
  }
  return current_m - dm_sum;
}
void Get_In_Position_Task::compare_configs(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_)
{
  std::vector<double> q_encoders = ctl_.robot().encoderValues();
  auto q_mbc = ctl_.robot().mbc().q;
  std::vector<double> q_dot_encoders = ctl_.robot().encoderVelocities();
  auto q_dot_mbc = ctl_.robot().mbc().alpha;
  auto ref_joint_order = ctl_.robot().refJointOrder();
  std::map<std::string, double> q_mbc_map;
  std::map<std::string, double> q_dot_mbc_map;
  std::map<std::string, double> q_encoders_map;
  std::map<std::string, double> q_dot_encoders_map;
  std::map<std::string, double> q_dot_encoders_derivative_map;
  std::map<std::string, double> q_dot_mbc_derivative_map;
  std::map<std::string, double> q_mbc_difference_map;
  std::map<std::string, double> q_encoders_difference_map;
  Eigen::VectorXd reorg_gradient = reorganized_gradient(gradient, ctl_);

  for(size_t i = 1; i < std::size(q_mbc); ++i)
  {
    if (q_mbc.at(i).empty()) {
      q_mbc_map[ctl_.robot().mb().joint(i).name()] = 0;
      q_dot_mbc_map[ctl_.robot().mb().joint(i).name()] = 0;
      q_mbc_difference_map[ctl_.robot().mb().joint(i).name()] = 0;
      q_dot_mbc_derivative_map[ctl_.robot().mb().joint(i).name()] = 0;

    }
    else {
      q_mbc_map[ctl_.robot().mb().joint(i).name()] = q_mbc.at(i).at(0);
      q_dot_mbc_map[ctl_.robot().mb().joint(i).name()] = q_dot_mbc.at(i).at(0);
      q_mbc_difference_map[ctl_.robot().mb().joint(i).name()] = q_mbc.at(i).at(0) - _old_mbc.q.at(i).at(0);
      q_dot_mbc_derivative_map[ctl_.robot().mb().joint(i).name()] = (q_mbc.at(i).at(0) - _old_mbc.q.at(i).at(0))/ctl_.solver().dt();
    }
  }

  for(size_t i = 0; i < std::size(q_dot_encoders); ++i)
  {
    q_encoders_map[ref_joint_order.at(i)] = q_encoders.at(i);
    q_dot_encoders_map[ref_joint_order.at(i)] = q_dot_encoders.at(i);
    q_encoders_difference_map[ref_joint_order.at(i)] = q_encoders.at(i) - _old_q_encoders.at(i);
    q_dot_encoders_derivative_map[ref_joint_order.at(i)] = (q_encoders.at(i) - _old_q_encoders.at(i))/ctl_.solver().dt();
  }

  mc_rtc::log::info("Q"); 

  for(const std::string &jointName : ref_joint_order)
  {
    mc_rtc::log::info("({}) q_mbc, q_encoders =  [{}, {}]", 
      jointName, 
      q_mbc_map.at(jointName),
      q_encoders_map.at(jointName)
      );
  }
  mc_rtc::log::info("---------------------"); 
  mc_rtc::log::info("Q_difference"); 

  for(const std::string &jointName : ref_joint_order)
  {
    mc_rtc::log::info("({}) q_mbc_difference, q_encoders_difference =  [{}, {}]", 
      jointName, 
      q_mbc_difference_map.at(jointName),
      q_encoders_difference_map.at(jointName)); 
  }
  mc_rtc::log::info("---------------------"); 
  mc_rtc::log::info("Q_DOT"); 

  uint8_t i = 0;
  for(const std::string &jointName : ref_joint_order)
  {
    mc_rtc::log::info("({}) q_dot_mbc, q_dot_mbc_der, q_dot_encoders, q_dot_encoders_der, grad_m =  [{}, {}, {}, {}, {}]", 
      jointName, 
      q_dot_mbc_map.at(jointName),
      q_dot_mbc_derivative_map.at(jointName),
      q_dot_encoders_map.at(jointName),
      q_dot_encoders_derivative_map.at(jointName),
      reorg_gradient(i, 0));
    ++i;
  }
  _old_q_encoders = q_encoders;
}
const double Get_In_Position_Task::emass_time_derivative_with_encoders(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_) const
{
  Eigen::VectorXd reorg_gradient = reorganized_gradient(gradient, ctl_);
  std::vector<double> q_dot_encoders = ctl_.robot().encoderVelocities();

  double dmdt = 0;
  for(size_t i = 0; i < std::size(q_dot_encoders); ++i)
  {
    dmdt += reorg_gradient(i, 0)*q_dot_encoders.at(i);
    // mc_rtc::log::info("({}) reorg_grad[{}] * qdot_encoders[{}] = {} * {}",ctl_.robot().refJointOrder().at(i), i, i,reorg_gradient(i, 0), q_dot_encoders.at(i) );
  }
  return dmdt;
}

const Eigen::VectorXd Get_In_Position_Task::reorganized_gradient(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_) const
{
  // Reorganize the gradient to multiply it by qdot from encoders
  Eigen::VectorXd res(std::size(ctl_.robot().encoderVelocities()), 1);
  res.setOnes();

  std::map<std::string, double> res_map;
  unsigned int j = 0;
  for(size_t i = 0; i < std::size(ctl_.robot().mbc().alpha); ++i)
  {
    if(std::size(ctl_.robot().mbc().alpha.at(i))!=1)
    {
      res_map[ctl_.robot().mb().joint(i).name()] = 0;
      continue;
    }
    res_map[ctl_.robot().mb().joint(i).name()] = gradient(j, 0);
    ++j;
  }
  for(size_t i = 0; i < std::size(ctl_.robot().refJointOrder()); ++i)
  {
    res(i, 0) = res_map.at(ctl_.robot().refJointOrder().at(i));
  }
  // mc_rtc::log::info("grad size = {}", std::size(gradient));
  // for(size_t i = 0; i < std::size(ctl_.robot().encoderVelocities()); ++i)
  // {
  //   if(i >= 0 && i < 6)
  //   {
  //     res(i, 0) = gradient(i+12, 0);
  //   }
  //   else if (i >= 6 && i < 12) 
  //   {  
  //     res(i, 0) = gradient(i, 0);
  //   }
  //   else if (i >= 12 && i < 17) 
  //   {
  //     res(i, 0) = gradient(i+6, 0);
  //   }
  //   else if (i >= 17 && i < 26) 
  //   {
  //     res(i, 0) = gradient(i+15, 0);
  //   }
  //   else if (i>= 26 && i < 35) 
  //   {
  //     res(i, 0) = 0;
  //   }
  //   else if (i >= 35 && i < 44) 
  //   {
  //     res(i, 0) = gradient(i-12, 0);
  //   }
  //   else if(i >= 44 && i < std::size(ctl_.robot().encoderVelocities()))
  //   {
  //     res(i, 0) = 0;
  //   }
  // }
  return res;
}

const double Get_In_Position_Task::emass_time_derivative_with_mbc_q_derivative(
  const Eigen::VectorXd &gradient, 
  mc_control::fsm::Controller & ctl_) const
{
  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);

  double dmdt = 0;
  double delta_q_i = 0;
  unsigned int j = 0;
  std::vector<std::vector<double>> q = _new_mbc.q;
  std::vector<std::vector<double>> old_q = _old_mbc.q;
  for (size_t i = 0; i < std::size(q); ++i)
  {
    if (std::size(q.at(i)) != 1)
    {
      //Ignore floating base (size = 6) and empty dofs (size = 0)
      continue;
    }    
    delta_q_i = q.at(i).at(0) - old_q.at(i).at(0);
    dmdt += gradient(j,0) * (delta_q_i / ctl.solver().dt());
    // mc_rtc::log::info("({}) grad[{}] * dq[{}]/dt = {} * {}",ctl.robot().mb().joint(i).name(), j, i, gradient(j, 0), dqi/ctl.solver().dt());
    j+=1;
  }
  return dmdt;
}
const double Get_In_Position_Task::emass_time_derivative_with_mbc_alpha(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_) const
{
  double dmdt = 0;
  unsigned int j = 0;
  std::vector<std::vector<double>> qdot = _new_mbc.alpha;
  
  for (size_t i = 0; i < std::size(qdot); ++i)
  {
    if (std::size(qdot.at(i)) != 1)
    {
      //Ignore floating base and empty dofs
      continue;
    }    
    dmdt += gradient(j, 0)*qdot.at(i).at(0);

    j+=1;
  }

  return dmdt;
}
const double Get_In_Position_Task::compute_effective_mass_with_mbc(
  rbd::MultiBodyConfig mbc, 
  mc_control::fsm::Controller & ctl_, 
  const Eigen::Vector3d &normal_vector) const{

  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);
    
  // If you dont put this line the gradient is 0 everywhere because M and J are not updating
  ctl.robot().forwardKinematics(mbc);                                               
                                                        

  rbd::MultiBody robot_mb = ctl_.robot().mb();
  // rbd::Jacobian jac(robot_mb, _hammer_head_frame_name, _jacobian_verbose_active);
  rbd::Jacobian jac(robot_mb, _hammer_head_frame_name);
  Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot_mb, mbc);

  Eigen::MatrixXd full_world_frame_jacobian(6, _nrdof);
  jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);

  const Eigen::MatrixXd M = _dynamicsConstraint->motionConstr().fd().H();
  // mc_rtc::log::info("M = {}", M);

  const Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);
  // writeEigenMatrixToCSV(linear_jacobian, "linear_jac.csv");
  // mc_rtc::log::info("Dim of M = {} x {}", M.cols(), M.rows());
  // mc_rtc::log::info("M = {}", M);

  const Eigen::Matrix3d LAMBDA = linear_jacobian*M.inverse()*linear_jacobian.transpose();

  // mc_rtc::log::info("Dim of  LINEAR LAMBDA = {} x {}", LINEAR_LAMBDA.cols(), LINEAR_LAMBDA.rows());

  return 1/(normal_vector.transpose()*LAMBDA*normal_vector);


}

// const double Get_In_Position_Task::compute_effective_mass_with_mbc(
//   rbd::MultiBodyConfig mbc, 
//   mc_control::fsm::Controller & ctl_, 
//   const Eigen::Vector3d &normal_vector) const{

//   auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
    
//   // If you dont put this line the gradient is 0 everywhere because M and J are not updating
//   ctl.robot().forwardKinematics(mbc);                                               
                                                        

//   rbd::MultiBody robot_mb = ctl_.robot().mb();
//   rbd::Jacobian jac(robot_mb, _hammer_head_frame_name, _jacobian_verbose_active);
//   Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot_mb, mbc);

//   Eigen::MatrixXd full_world_frame_jacobian(6, _nrdof);
//   jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);

//   const Eigen::MatrixXd M = _dynamicsConstraint->motionConstr().fd().H();

//   const Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);
//   // writeEigenMatrixToCSV(linear_jacobian, "linear_jac.csv");
//   // mc_rtc::log::info("Dim of M = {} x {}", M.cols(), M.rows());
//   // mc_rtc::log::info("M = {}", M);

//   // const Eigen::Matrix3d LAMBDA = linear_jacobian*M.inverse()*linear_jacobian.transpose();
//   const Eigen::MatrixXd LAMBDA_INVERSE = (full_world_frame_jacobian*M*full_world_frame_jacobian.transpose()).inverse();

//   const Eigen::MatrixXd LINEAR_LAMBDA_INVERSE = LAMBDA_INVERSE.block(0, 0, 3, 3);
//   // mc_rtc::log::info("Dim of  LINEAR LAMBDA = {} x {}", LINEAR_LAMBDA.cols(), LINEAR_LAMBDA.rows());

//   // return 1/(normal_vector.transpose()*LAMBDA*normal_vector);
//   return 1/(normal_vector.transpose()*LINEAR_LAMBDA_INVERSE*normal_vector);

// }

const rbd::MultiBodyConfig Get_In_Position_Task::encoders_values_to_mbc(
  const std::vector<double> &encoderValues,
  mc_control::fsm::Controller & ctl_) const
{
  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);
  rbd::MultiBodyConfig mbc_res = ctl.robot().mbc();
  std::vector<double> q_encoders = encoderValues;
  std::map<std::string, std::vector<double>> q_mbc_map;
  std::vector<std::string> ref_joint_order = ctl.robot().refJointOrder();
  std::map<std::string, double> q_encoders_map;
  std::map<std::string, std::vector<double>> res;
  
      
  //Convert the encoderValues vector into some mbc.q vector
  for(size_t i = 0; i < std::size(q_encoders); ++i)
  {
    q_encoders_map[ref_joint_order.at(i)] = q_encoders.at(i);
  }

  for(size_t i = 0; i < std::size(mbc_res.q); ++i)
  {
    q_mbc_map[ctl_.robot().mb().joint(i).name()] = mbc_res.q.at(i);
  }

  for(size_t i = 0; i < std::size(mbc_res.q); ++i)
  {
    std::string jointName = ctl_.robot().mb().joint(i).name();
    try{
      if (std::size(q_mbc_map.at(jointName)) != 1)
      {
        res[jointName] = q_mbc_map.at(jointName);
      }
      else
      {
        res[jointName] = {q_encoders_map.at(jointName)};
      }
    }
    catch (std::out_of_range){
      res[jointName] = q_mbc_map.at(jointName);
    }
    mbc_res.q[i] = res.at(jointName);
  }
  return mbc_res;
}
const double Get_In_Position_Task::compute_effective_mass_with_encoders(
  const std::vector<double> &encoderValues,
  mc_control::fsm::Controller & ctl_, 
  const Eigen::Vector3d &normal_vector) const{

  HammeringTaskNew &ctl = static_cast<HammeringTaskNew &>(ctl_);
  rbd::MultiBodyConfig mbc = encoders_values_to_mbc(encoderValues, ctl);
  ctl_.robot().forwardKinematics(mbc);


  rbd::MultiBody robot_mb = ctl_.robot().mb();
  // rbd::Jacobian jac(robot_mb, _hammer_head_frame_name, _jacobian_verbose_active);
  rbd::Jacobian jac(robot_mb, _hammer_head_frame_name);
  Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot_mb, mbc);

  Eigen::MatrixXd full_world_frame_jacobian(6, _nrdof);
  jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);

  // _dynamicsConstraint->motionConstr().fd_non_const().computeH(robot_mb, mbc);
  Eigen::MatrixXd M = ctl.dynamicsConstraint->motionConstr().fd().H();
  Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);

  Eigen::MatrixXd M_inverse = M.inverse();
  Eigen::Matrix3d LAMBDA = linear_jacobian*M_inverse*linear_jacobian.transpose();
  return 1/(normal_vector.transpose()*LAMBDA*normal_vector);
}

const Eigen::VectorXd Get_In_Position_Task::compute_emass_gradient_backward_difference_mbc(
  const rbd::MultiBodyConfig &mbc, 
  mc_control::fsm::Controller &ctl_, 
  const Eigen::Vector3d &normal_vector) const{

  const double epsilon = 1E-6;
  Eigen::VectorXd grad(_nrdof, 1);
  grad.setOnes();
  double m_q = compute_effective_mass_with_mbc(mbc, ctl_, normal_vector);
  double backward_effective_mass = 0;
  
  unsigned int j = 0;
  rbd::MultiBodyConfig backward_mbc;
  for(size_t i = 0; i <  std::size(mbc.q); ++i)
  { 
    if(mbc.q.at(i).size() != 1)
    {
      // Ignore floating base and fixed joints
      continue;
    }
    // dqi = mbc.q.at(i).at(0) - _old_mbc.q.at(i).at(0);
    backward_mbc = mbc;
    backward_mbc.q.at(i).at(0) -= epsilon;

    //Finite differences
    backward_effective_mass = compute_effective_mass_with_mbc(backward_mbc, ctl_, normal_vector);
    grad(j,0) = (m_q - backward_effective_mass)/epsilon;
    j+=1;
      
      // mc_rtc::log::info("forward_mbc_q[i] = {}", forward_mbc.q.at(i).at(0));
      // mc_rtc::log::info("backward_mbc_q[i] = {}", backward_mbc.q.at(i).at(0));
      // mc_rtc::log::info("---------------------");
      // mc_rtc::log::info("grad[{}] = {}", 6+j, grad_res);
      // backward_mbc.q.at(i).at(0) = mbc.q.at(i).at(0);
  }
  return grad;
}

const Eigen::VectorXd Get_In_Position_Task::compute_emass_gradient_three_point_backward_difference_mbc(
  const rbd::MultiBodyConfig &mbc, 
  mc_control::fsm::Controller &ctl_, 
  const Eigen::Vector3d &normal_vector) const{

  const double epsilon = 1E-6;
  Eigen::VectorXd grad(_nrdof, 1);
  grad.setOnes();
  double backward_effective_mass = 0;
  double m_q = compute_effective_mass_with_mbc(mbc, ctl_, normal_vector);
  
  unsigned int j = 0;
  rbd::MultiBodyConfig p1;
  rbd::MultiBodyConfig p2;

  rbd::MultiBodyConfig p3;
  rbd::MultiBodyConfig p4;

  double first_term = 0;
  double second_term = 0;


  for(size_t i = 0; i <  std::size(mbc.q); ++i)
  { 
    if(mbc.q.at(i).size() != 1)
    {
      // Ignore floating base and fixed joints
      continue;
    }
    // dqi = mbc.q.at(i).at(0) - _old_mbc.q.at(i).at(0);
    p1 = mbc;
    p2 = mbc;
    p3 = mbc;
    p4 = mbc;

    p1.q.at(i).at(0) -= 2*epsilon;
    p2.q.at(i).at(0) -= epsilon;

    first_term = compute_effective_mass_with_mbc(p1, ctl_, normal_vector);
    second_term = compute_effective_mass_with_mbc(p2, ctl_, normal_vector);

    //Finite differences
    backward_effective_mass = first_term - 4*second_term + 3*m_q;
    grad(j,0) = backward_effective_mass/(2*epsilon);
    j+=1;
      
      // mc_rtc::log::info("forward_mbc_q[i] = {}", forward_mbc.q.at(i).at(0));
      // mc_rtc::log::info("backward_mbc_q[i] = {}", backward_mbc.q.at(i).at(0));
      // mc_rtc::log::info("---------------------");
      // mc_rtc::log::info("grad[{}] = {}", 6+j, grad_res);
      // backward_mbc.q.at(i).at(0) = mbc.q.at(i).at(0);
  }
  return grad;
}

const Eigen::VectorXd Get_In_Position_Task::compute_emass_gradient_central_difference_mbc(
                                                            const rbd::MultiBodyConfig &mbc, 
                                                             mc_control::fsm::Controller &ctl_, 
                                                            const Eigen::Vector3d &normal_vector) const{
    auto & ctl = static_cast<HammeringTaskNew &>(ctl_);

    double dqi = 0;
    Eigen::VectorXd grad(_nrdof, 1);
    grad.setOnes();

    rbd::MultiBodyConfig forward_mbc;
    rbd::MultiBodyConfig backward_mbc;

    double forward_effective_mass = 0;
    double backward_effective_mass = 0;
    unsigned int j = 0;
    // ctl.robot().forwardKinematics(mbc);
    for(size_t i = 0; i <  std::size(mbc.q); ++i)
    { 
      if(mbc.q.at(i).size() != 1)
      {
        // Ignore floating base (size = 6) and fixed joints (size = 0)
        continue;
      }
      dqi = mbc.q.at(i).at(0) - _old_mbc.q.at(i).at(0);
      forward_mbc = mbc;
      backward_mbc = mbc;
      forward_mbc.q.at(i).at(0) += dqi;
      backward_mbc.q.at(i).at(0) -= dqi;

      //Finite differences - Central differences
      forward_effective_mass = compute_effective_mass_with_mbc(forward_mbc, 
                                                                ctl, 
                                                                  normal_vector);
      backward_effective_mass = compute_effective_mass_with_mbc(backward_mbc, 
                                                                  ctl, 
                                                                  normal_vector);

      grad(j,0) = (forward_effective_mass - backward_effective_mass)/(2*dqi);
      j+=1;
    }
  
  return grad;
}

const Eigen::VectorXd Get_In_Position_Task::compute_emass_gradient_central_difference_encoders(
                                                            const std::vector<double> &encoderValues, 
                                                             mc_control::fsm::Controller &ctl_, 
                                                            const Eigen::Vector3d &normal_vector) const{

    // NOT FINISHED !!!!
    auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
    double dqi = 0; 
    Eigen::VectorXd grad(_nrdof, 1);
    grad.setOnes();

    std::vector<std::string> unusable_dofs;
    for (size_t i = 0; i < std::size(ctl_.robot().mbc().q); ++i)
    {
      if (std::size(ctl_.robot().mbc().q.at(i)) != 1)
      {
        unusable_dofs.push_back(ctl_.robot().mb().joint(i).name());
      }
    }
    // std::map<std::string, double> encoder_values_map;
    // for (size_t i = 0; i < std::size(encoderValues); ++i)
    // {
    //   encoder_values_map[ctl_.robot().refJointOrder().at(i)] = encoderValues.at(i);
    // }
    // std::vector<double> reorg_encoder_values;
    // for (size_t i = 0; i < std::size(usable_dofs); ++i)
    // {
    //   reorg_encoder_values.at(i) = encoder_values_map.at(usable_dofs.at(i));
    // }    

    double forward_effective_mass = 0;
    double backward_effective_mass = 0;

    for(size_t i = 0; i < 6; ++i)
    {
      //We suppose that the floating base does not move, hence the first 6 DOFs gradient wrt to q are 0
      grad(i,0) = 0;
    }
    unsigned int j = 6; // Skipping the first 6 rows representing the floating base
    for(size_t i = 0; i <  std::size(encoderValues); ++i)
    { 
      // What dof i am touching : if it is an unactuated dof i should skip
      if (std::find(unusable_dofs.begin(), 
                      unusable_dofs.end(), 
                      ctl_.robot().refJointOrder().at(i)) != unusable_dofs.end())
                      //refJointOrder should be the same as the mbc order ?
      {
        continue;
      }
      std::vector<double> forward_encoder_values = encoderValues;
      std::vector<double> backward_encoder_values = encoderValues;
      
      dqi = encoderValues.at(i) - _old_q_encoders.at(i);
      forward_encoder_values.at(i) += dqi;
      backward_encoder_values.at(i) -= dqi;

      //Finite differences - Central differences
      forward_effective_mass = compute_effective_mass_with_encoders(forward_encoder_values, 
                                                                                ctl_, 
                                                                                  normal_vector);
      backward_effective_mass = compute_effective_mass_with_encoders(backward_encoder_values, 
                                                                                ctl_, 
                                                                                  normal_vector);
      grad(j,0) = (forward_effective_mass - backward_effective_mass)/(2*dqi);
      j+=1;
      
    }
  
  return grad;
}
const double Get_In_Position_Task::compute_effective_mass_naive(rbd::MultiBodyConfig mbc, mc_control::fsm::Controller & ctl_) const
{
  // ctl_.robot().forwardKinematics(mbc);
  // auto robot_mb = ctl_.robot().mb();
  // _dynamicsConstraint->motionConstr().fd_non_const().computeH(robot_mb, q_test);
  // double delta_psi = 0.01; // rad

  // std::cout << "nrDOF = " << _nrdof << std::endl;
  // std::cout << "size = " << std::size(qf.q) << std::endl;
  // int n = 0;
  std::cout << "qf = {"  << std::endl;

  // Store only the actuated dofs in Q
  // std::vector<double> Q = {};
  for (size_t i = 0; i < std::size(mbc.q); ++i)
  {
    std::cout << ctl_.robot().mb().joint(i).name()<< " : " ; 
    std::cout << "{";
    for(size_t j = 0; j < std::size(mbc.q.at(i)); ++j)
    {
      
      std::cout << mbc.q.at(i).at(j) << ", ";
  //     n+=1;
  //     Q.push_back(qf.q.at(i).at(j));
    }
    std::cout << "}" << std::endl;
  }
  std::cout << "}" << std::endl;
  // std::cout << "n = " << n << std::endl;

  // std::cout << "Q = {"  << std::endl;
  // for (size_t i = 0; i < std::size(Q); ++i){
  //   std::cout << Q.at(i) << std::endl;
  // }
  // std::cout << "}"  << std::endl;
  // Eigen::MatrixXd M = _dynamicsConstraint->motionConstr().fd().H();
  //   // std::cout << "M  = \n" << M << "\n";
  // Eigen::MatrixXd M_inverse = M.inverse();
  // // std::cout << "Creating my Jacobian" << std::endl;
  // rbd::Jacobian jac(robot_mb, _hammer_head_frame_name, _jacobian_verbose_active);
  // auto world_frame_jacobian = jac.jacobian(robot_mb,mbc);
  // // std::cout << "Created world frame jacobian" << std::endl;

  // Eigen::MatrixXd full_world_frame_jacobian(6, ctl_.robot().mb().nrDof());
  // jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);
  // // std::cout << "Projected my world frame Jacobian" << std::endl;
  
  // Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);
  // std::cout << "Created linear jacobian" << std::endl;

  // Eigen::MatrixXd angular_jacobian = full_world_frame_jacobian.bottomRows(3);
  // std::cout << "Created angular jacobian" << std::endl;

  // std::cout << "dim angular jacobian = " << angular_jacobian.rows() << " x " << angular_jacobian.cols() << std::endl;
  // Eigen::MatrixXd pseudo_inverted_angular_jacobian = angular_jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(angular_jacobian.rows(), angular_jacobian.rows()));
  // std::cout << "Created pseudo inverse" << std::endl;
  
  // auto pseudo_inverted_angular_jacobian_third_column = pseudo_inverted_angular_jacobian.col(3 - 1);
  // std::cout << "Created pseudo inverse angular jac" << std::endl;

  // Eigen::MatrixXd delta_x_omega(3, 1);
  // std::cout << "Created delta x omega" << std::endl;

  // delta_x_omega << 0, 0, delta_psi;

  // Eigen::MatrixXd delta_q =  pseudo_inverted_angular_jacobian * delta_x_omega;
  // std::cout << "Created delta q" << std::endl;e

  // std::cout << "JointConfig = { " << std::endl;
  // for(size_t i = 0; i < std::size(qf.jointConfig); ++i)
  // {
  //   std::cout << qf.jointConfig.at(i) << std::endl;
  // }
  // std::cout << "}" << std::endl;

  // std::cout << "delta q = " << delta_q << std::endl;

  // auto incremented_robot_config = qf.q;
  // incremented_robot_config.at(37);
  
  // std::cout << "Joint " << ctl_.robot().mb().jointIndexByName("LWRY") << std::endl;
  // std::cout << "Joint value at 37 : " << Q.at(37 + 1) << std::endl;

  
  // std::cout << M_inverse << std::endl;
  // std::cout << "Created M inverse" << std::endl;
  // std::cout << "Size M^-1 = " << M_inverse.rows() << " x " << M_inverse.cols() << std::endl;
  // writeEigenMatrixToCSV(M, "MassMatrixMcRTC.csv");
  // writeEigenMatrixToCSV(full_world_frame_jacobian, "full_world_frame_jacobian.csv");
  // std::cout << M << std::endl;
  
  
  
  double inner_sum = 0;
  double J_nu_3_j = 0;
  double J_nu_3_k = 0;
  double M_inverse_j_k = 0;
  
  double outer_sum = 0;
  double effective_mass = 0;
  double delta_psi_tot = 0;
  // auto q_start = qf;
  // std::cout << "Starting double sum" << std::endl;
  
  // while(delta_psi_tot < 2*M_PI)
  // {
    // std::cout << "delta_psi_tot = " << delta_psi_tot << std::endl;
    //Create everything

    // std::cout << M_inverse.row(0) << std::endl;
    // world_frame_jacobian = jac.jacobian(robot_mb,q_test);

    // Eigen::MatrixXd full_world_frame_jacobian(6, robot_mb.nrDof());
    // jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);
    
    // Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);

    // Eigen::MatrixXd angular_jacobian = full_world_frame_jacobian.bottomRows(3);

    // std::cout << "dim angular jacobian = " << angular_jacobian.rows() << " x " << angular_jacobian.cols() << std::endl;
    // Eigen::MatrixXd pseudo_inverted_angular_jacobian = angular_jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(angular_jacobian.rows(), angular_jacobian.rows()));
    
    // pseudo_inverted_angular_jacobian_third_column = pseudo_inverted_angular_jacobian.col(3 - 1);

    // Eigen::MatrixXd delta_x_omega(3, 1);

    // Eigen::MatrixXd pseudo_inverted_full_jac = full_world_frame_jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(full_world_frame_jacobian.rows(), full_world_frame_jacobian.rows()));
    // delta_x_omega << 0, 0, delta_psi;
    // Eigen::MatrixXd delta_x(6, 1);
    // delta_x << 0, 0, 0, 0, 0, delta_psi;
    // Eigen::MatrixXd delta_q =  pseudo_inverted_angular_jacobian * delta_x_omega;
    // Eigen::MatrixXd delta_q =  pseudo_inverted_full_jac* delta_x;


    // Compute Outer sum
    // outer_sum = 0;
    // for(int k=0; k <= _nrdof - 1 ; ++k)
    // {
    //   // Inner sum
    //   inner_sum = 0;
    //   for(int j=0; j <= _nrdof - 1; ++j)
    //   { 
    //     J_nu_3_j = linear_jacobian(3-1,j);
    //     // std::cout << "Computed linear jac (3-1, "  << j  << ") = " << linear_jacobian(3-1, j) << std::endl;
    //     M_inverse_j_k = M_inverse(j, k);
    //     // std::cout << "M_inverse (" << j << "," << k << ") = " << M_inverse_j_k << std::endl;
    //     inner_sum += J_nu_3_j * M_inverse_j_k;
    //   }
    //   J_nu_3_k = linear_jacobian(3-1, k);
    //   outer_sum += inner_sum*J_nu_3_k;
    // }

    // std::cout << "Testing effective mass : "<< 1/outer_sum << std::endl;
    // if(1/outer_sum > effective_mass)
    // {
    //   effective_mass = 1/outer_sum;
    // }

    // std::cout << "deltaq =  " << delta_q << std::endl;

    // printConfig(q_test, "q_test before = {");        
    // increment q_test by delta q
    // int L = 0;
    // int m = 0;
    // for (size_t i = 0; i < std::size(q_test.q); ++i)
    // {
    //   for(size_t j = 0; j < std::size(q_test.q.at(i)); ++j)
    //   {
    //     q_test.q.at(i).at(j) += delta_q(j+L, 0);
    //     m += 1;
    //   }
    //   L += m;
    //   m = 0;
    // }

    // printConfig(q_test, "q_test after = {");        
  //   delta_psi_tot += delta_psi;
  // }
  return 1/outer_sum;
}

void Get_In_Position_Task::printConfig(rbd::MultiBodyConfig mbc, std::string string) const
{
    std::cout << string << std::endl;
    for (size_t i = 0; i < std::size(mbc.q); ++i)
    {
      for(size_t j = 0; j < std::size(mbc.q.at(i)); ++j)
      {
        std::cout << mbc.q.at(i).at(j) <<"," << std::endl;
      }
    }
    std::cout << "}" << std::endl;
}

void Get_In_Position_Task::writeEigenMatrixToCSV(const Eigen::MatrixXd& matrix, const std::string& filename) const {
    // std::cout << "entering write eigen function" << std::endl;
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file!" << std::endl;
        return;
    }

    // std::cout << "For loop writeEigen begin" << std::endl;
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            file << matrix(i, j);
            if (j < matrix.cols() - 1) file << ",";  // Separate columns by commas
        }
        file << "\n";  // Newline for each row
    }
    file.close();
    // std::cout << "Matrix written to " << filename << std::endl;
}
void Get_In_Position_Task::writeVectorToCSV(const std::vector<double> & vec, const std::string & filename) const 
{
    std::ofstream file(filename);
    if(!file.is_open())
    {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }

    for(size_t i = 0; i < vec.size(); ++i)
    {
        file << vec[i];
        if(i != vec.size() - 1)
            file << ",";  // comma between values
    }

    file << "\n";  // new line at the end
    file.close();
}

const Eigen::Matrix3d Get_In_Position_Task::roll_rotation_nail_frame(const double &angle) const
{
  Eigen::Matrix3d res;
  res << 1,         0,                0,
         0,    cos(angle),    -sin(angle),
         0,    sin(angle),    cos(angle);
  return res;
}

const Eigen::Matrix3d Get_In_Position_Task::pitch_rotation_nail_frame(const double &angle) const
{
  Eigen::Matrix3d res;
  res << cos(angle),     0,       sin(angle),
               0,           1,            0,
         -sin(angle),     0,      cos(angle);
  return res;
  
}

const Eigen::Matrix3d Get_In_Position_Task::yaw_rotation_nail_frame(const double &angle) const
{
  Eigen::Matrix3d res;
  res<< cos(angle),   sin(angle),       0,
        -sin(angle),  cos(angle),       0,
              0,                0,           1;
  return res;
}


void Get_In_Position_Task::load_parameters()
{
  _nail_robot_name = "nail";
  _main_robot_name = "hrp5_p";
  // ------------------------ Loading timestep ---------------------------
  std::string timestep_key = "timestep";
  _timestep = _config(timestep_key);

  // ------------------------ Loading gui parameters ---------------------------

  std::string gui_key = "gui";
  std::string stop_hammering_button_name_key = "stop_hammering_button_name";
  _config(gui_key)(stop_hammering_button_name_key, _stop_hammering_button_name);

  // ------------------------ Loading quality of life parameters ---------------------------

  std::string qol_key = "quality_of_life";
  std::string jacobian_verbose_active_key = "jacobian_verbose_active";
  std::string bezier_curve_verbose_active_key = "bezier_curve_verbose_active";
  _bezier_curve_verbose_active = _config(qol_key)(bezier_curve_verbose_active_key);
  _jacobian_verbose_active = _config(qol_key)(jacobian_verbose_active_key);



  // ------------------------ Loading frames ---------------------------

  std::string frames_key = "frames";
  std::string hammerhead_frame_key = "Hammer_Head";
  std::string nail_frame_key = "nail";
 _config(frames_key)(hammerhead_frame_key, _hammer_head_frame_name);
 _config(frames_key)(nail_frame_key, _nail_frame_name);

  // ------------------------ Loading magic values ---------------------------

  std::string magic_values_key = "magic_values";
  _magic_max_control_point_height = _config(magic_values_key)("max_control_point_height");
  _magic_bezier_curve_max_duration = _config(magic_values_key)("bezier_curve_max_duration");
  _magic_task_stiffness = _config(magic_values_key)("task_stiffness");
  _magic_task_weight = _config(magic_values_key)("task_weight");
  _magic_epsilon = _config(magic_values_key)("epsilon");
  _posture_task_weight = _config(magic_values_key)("posture_task_weight");
  _effective_mass_maximization_task_weight = _config(magic_values_key)("effective_mass_maximization_task_weight");

  _magic_oriWp_time = _config(magic_values_key)("oriWp_time");



  // ------------------------ Loading init and start velocities, accelerations and jerks ---------------------------

  std::string curve_constraints_key = "curve_constraints";
  std::string linear_velocity_key = "linear_velocity";
  std::string linear_acceleration_key = "linear_acceleration";
  std::string linear_jerk_key = "linear_jerk";
  std::string x_key = "x";
  std::string y_key = "y";
  std::string z_key = "z";

  std::string init_key = "init";
  std::string end_key = "end";

  _constr.init_vel.x() = _config(curve_constraints_key)(linear_velocity_key)(x_key)(init_key);
  _constr.init_vel.y() = _config(curve_constraints_key)(linear_velocity_key)(y_key)(init_key);
  _constr.init_vel.z() = _config(curve_constraints_key)(linear_velocity_key)(z_key)(init_key);

  _constr.init_acc.x() = _config(curve_constraints_key)(linear_acceleration_key)(x_key)(init_key);
  _constr.init_acc.y() = _config(curve_constraints_key)(linear_acceleration_key)(y_key)(init_key);
  _constr.init_acc.z() = _config(curve_constraints_key)(linear_acceleration_key)(z_key)(init_key);

  _constr.init_jerk.x() = _config(curve_constraints_key)(linear_jerk_key)(x_key)(init_key);
  _constr.init_jerk.y() = _config(curve_constraints_key)(linear_jerk_key)(y_key)(init_key);
  _constr.init_jerk.z() = _config(curve_constraints_key)(linear_jerk_key)(z_key)(init_key);

  _constr.end_vel.x() = _config(curve_constraints_key)(linear_velocity_key)(x_key)(end_key);
  _constr.end_vel.y() = _config(curve_constraints_key)(linear_velocity_key)(y_key)(end_key);
  _constr.end_vel.z() = _config(curve_constraints_key)(linear_velocity_key)(z_key)(end_key);

  _constr.end_acc.x() = _config(curve_constraints_key)(linear_acceleration_key)(x_key)(end_key);
  _constr.end_acc.y() = _config(curve_constraints_key)(linear_acceleration_key)(y_key)(end_key);
  _constr.end_acc.z() = _config(curve_constraints_key)(linear_acceleration_key)(z_key)(end_key);

  _constr.end_jerk.x() = _config(curve_constraints_key)(linear_jerk_key)(x_key)(end_key);
  _constr.end_jerk.y() = _config(curve_constraints_key)(linear_jerk_key)(y_key)(end_key);
  _constr.end_jerk.z() = _config(curve_constraints_key)(linear_jerk_key)(z_key)(end_key);
}

EXPORT_SINGLE_STATE("Get_In_Position_Task", Get_In_Position_Task)