#include "vf_controller/vf_controller.h"
#include <boost/concept_check.hpp>

using namespace vf_controller;
using namespace std;
using namespace Eigen;
using namespace XmlRpc;

bool VFController::init(sybot_hw_interface::SybotHwInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
  
    assert(hw != NULL);

    hw_ = hw;

    cart_size_ = hw_->getNdofCart();
    Nodf_kin_ = hw_->getNdof();

    // Resize
    torques_cmd_.resize(Nodf_kin_);
    joint_pos_status_.resize(Nodf_kin_);
    joint_vel_status_.resize(Nodf_kin_);
    cart_pose_status_.resize(cart_size_);
    cart_vel_status_.resize(cart_size_);
    f_vm_.resize(cart_size_);
    jacobian_tmp_.resize(cart_size_);
    for (int i = 0; i < cart_size_; ++i)
        jacobian_tmp_[i].resize(Nodf_kin_);

    jacobian_.resize(cart_size_,Nodf_kin_);

    // Retrain the joint handles
    joints_.resize(Nodf_kin_);
    std::string joint_name;
    for (int i = 0; i < Nodf_kin_; ++i)
    {
      joint_name = hw_->getJointName(i);
      // Get a joint handle
      try
      {

      hw_eff_interface_ = hw_->get<hardware_interface::EffortJointInterface>();

      joints_[i] = hw_eff_interface_->getHandle(joint_name);

      ROS_DEBUG_STREAM("Found joint '" << joint_name << "' in the '" <<
			    getHardwareInterfaceType() << "' hardware interface.");
      }
      catch (std::exception const &e)
      {
	  ROS_ERROR_STREAM("Could not find joint '" << joint_name << "' in the '" <<
			    getHardwareInterfaceType() << "' hardware interface:" << e.what());
	  return false;
      }
      catch (...)
      {
	  ROS_ERROR_STREAM("Could not find joint '" << joint_name << "' in the '" <<
			    getHardwareInterfaceType() << "' hardware interface.");
	  return false;
      }
    }
    
    return true;
}

void VFController::update(const ros::Time& time, const ros::Duration& period)
{

    hw_->getJacobian(jacobian_tmp_);

    for (int i = 0; i<Nodf_kin_; i++)
    {
      joint_pos_status_(i) = joints_[i].getPosition();
      joint_vel_status_(i) = joints_[i].getVelocity();

    }

    for (int i = 0; i<cart_size_; i++)
    {
        cart_pose_status_(i) = hw_->getXValue(i);
        cart_vel_status_(i) = hw_->getXdValue(i);
        for (int j = 0; j<Nodf_kin_; j++)
            jacobian_(i,j) = jacobian_tmp_[i][j];
    }

    // Update the virtual mechanisms
    mechanism_manager_.Update(cart_pose_status_,cart_vel_status_,period.toSec(),f_vm_);

    torques_cmd_.noalias() = jacobian_.transpose() * f_vm_;
    
    for (int i = 0; i<Nodf_kin_; i++)
    {
        joints_[i].setCommand(torques_cmd_(i));
    }


}







/*PLUGINLIB_DECLARE_CLASS(vf_controller,
                        VFController,
                        vf_controller::VFController,
                        controller_interface::ControllerBase)*/

//PLUGINLIB_EXPORT_CLASS(controller::Controller, controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(vf_controller::VFController, controller_interface::ControllerBase);
