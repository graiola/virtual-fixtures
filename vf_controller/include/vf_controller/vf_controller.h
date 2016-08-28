/**
 * @file   vf_controller.cpp
 * @brief  Plugin controller for ros-control.
 * @author Gennaro Raiola
 *
 * This file is part of virtual-fixtures, a set of libraries and programs to create
 * and interact with a library of virtual guides.
 * Copyright (C) 2014-2016 Gennaro Raiola, ENSTA-ParisTech
 *
 * virtual-fixtures is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * virtual-fixtures is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with virtual-fixtures.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VF_CONTROLLER_H
#define VF_CONTROLLER_H

////////// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include <sybot_hw_interface/sybot_hw_interface.h>
#include <controller_interface/controller.h>
#include <forward_command_controller/forward_command_controller.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>

////////// Mechanism Manager
#include <mechanism_manager/mechanism_manager.h>

namespace vf_controller
{
  typedef std::vector<hardware_interface::JointHandle> joints_t;
  typedef Eigen::Quaternion<double> quaternion_t;

class VFController : public controller_interface::Controller<sybot_hw_interface::SybotHwInterface>
{
public:
  VFController():hw_(NULL){}
  //~VFController(){if(hw_!=NULL) delete hw_;}
  
  bool init(sybot_hw_interface::SybotHwInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
  void starting(const ros::Time& time) { }
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time) { }
  
private:
  
  mechanism_manager::MechanismManager mechanism_manager_;
  
  joints_t joints_;
  
  Eigen::VectorXd torques_cmd_;
  Eigen::VectorXd joint_pos_status_;
  Eigen::VectorXd joint_vel_status_;
  Eigen::VectorXd cart_pose_status_;
  Eigen::VectorXd cart_pose_with_quat_status_;
  Eigen::VectorXd cart_vel_status_;
  Eigen::VectorXd f_vm_;

  Eigen::MatrixXd jacobian_;
  mat_t jacobian_tmp_;
  
  sybot_hw_interface::SybotHwInterface* hw_;

  hardware_interface::EffortJointInterface* hw_eff_interface_;

  int cart_size_;
  int Nodf_kin_;
  


};

}

#endif
