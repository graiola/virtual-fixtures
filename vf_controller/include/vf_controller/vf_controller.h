#ifndef VF_CONTROLLER_H
#define VF_CONTROLLER_H

////////// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include <controller_interface/controller.h>
#include <forward_command_controller/forward_command_controller.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>

////////// Mechanism Manager
#include <mechanism_manager/mechanism_manager.h>

////////// KDL_KINEMATICS
#include <kdl_kinematics/kdl_kinematics.h>

namespace vf_controller
{
  typedef Eigen::JacobiSVD<Eigen::MatrixXd> svd_t;
  typedef std::vector<hardware_interface::JointHandle> joints_t;
  
/*class VFHardwareInterface : public hardware_interface::HardwareInterface
{
public:
  VFHardwareInterface()
  {
      
  }
};*/

class VFController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  VFController():kin_(NULL){}
  ~VFController(){if(kin_!=NULL) delete kin_;}
  
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
  void starting(const ros::Time& time) { }
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time) { }
  
private:
  
  kdl_kinematics::KDLKinematics* kin_;
  mechanism_manager::MechanismManager mechanism_manager_;
  
  joints_t joints_;
  
  Eigen::VectorXd torques_cmd_;
  Eigen::VectorXd joint_pos_status_;
  Eigen::VectorXd joint_vel_status_;
  Eigen::VectorXd cart_pos_status_;
  Eigen::VectorXd cart_vel_status_;
  Eigen::VectorXd f_vm_;
  
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd jacobian_t_;               
  
  int cart_size_;
  int Nodf_kin_;
  
  

};

}

#endif