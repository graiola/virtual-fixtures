#ifndef VF_CONTROLLER_H
#define VF_CONTROLLER_H

////////// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include <controller_interface/controller.h>
#include <forward_command_controller/forward_command_controller.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf_conversions/tf_kdl.h>

////////// Mechanism Manager
#include <mechanism_manager/mechanism_manager.h>
#include <vf_controller/kinematics_base_controller.h>
#include <vf_controller/PoseRPY.h>

namespace vf_controller
{

class VFController : public controller_interface::KinematicsBaseController<hardware_interface::EffortJointInterface>
{
public:
  VFController(){}
  ~VFController(){if(mechanism_manager_!=NULL) delete mechanism_manager_;}
  
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &nh);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time) { }

  void interaction(const vf_controller::PoseRPY::ConstPtr &msg);
  void publish_frame(const KDL::Frame& f);
  
private:
  
  ros::Subscriber sub_interaction_;
  boost::shared_ptr< realtime_tools::RealtimePublisher< geometry_msgs::PoseStamped > > rt_frame_pub_;

  mechanism_manager::MechanismManager* mechanism_manager_;

  KDL::Frame x_frame_, //current robot frame
  x_ext_;	//external refenrence frame
  KDL::Twist x_err_;  //error computed in respect of the external reference frame
  KDL::JntArrayAcc joints_msr_;
  KDL::JntArray torques_cmd_; //computed torques
  KDL::JntArray G_; //gravity
  //KDL::JntArray C_;	//coriolis
  KDL::Jacobian J_;	//Jacobian
  KDL::JntSpaceInertiaMatrix M_; //Inertia matrix
  KDL::JntArray cg_cmd_; //compensation torques

  Eigen::VectorXd x_,//current robot pose (x y z, TODO)
  x_dot_,  //current robot velocity
  f_vm_,  //current total force from mechanism manager
  f_ext_; //current external force

  bool cg_comp_on_, // Gravity compensation ON/OFF
  interaction_on_; // External interaction ON/OFF

};

}

#endif
