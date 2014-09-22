#ifndef VF_CONTROLLER_H
#define VF_CONTROLLER_H

#include <controller_interface/controller.h>
# include <forward_command_controller/forward_command_controller.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>

namespace vf_controller
{

/*class VFHardwareInterface : public hardware_interface::HardwareInterface
{
public:
  VFHardwareInterface()
  {
      
  }
};*/

class VFController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  VFController() { std::cout<<"HERE"<<std::endl; getchar(); }

  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n) { return true; }
  void starting(const ros::Time& time) { }
  void update(const ros::Time& time, const ros::Duration& period) { std::cout<<"HELLO BOY"<<std::endl;}
  void stopping(const ros::Time& time) { }
};

}

#endif