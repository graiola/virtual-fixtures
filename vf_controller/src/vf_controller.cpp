#include "vf_controller/vf_controller.h"
#include <boost/concept_check.hpp>

using namespace vf_controller;
using namespace std;
using namespace kdl_kinematics;
using namespace Eigen;
using namespace XmlRpc;

bool VFController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
  
    XmlRpcValue damp_max, epsilon, root, end_effector, orientation; 
    std::string param_name = "kinematics/damp_max";
    if (!controller_nh.getParam(param_name, damp_max))
    {
	ROS_ERROR_STREAM("No damp_max given (expected namespace: " + param_name + ").");
	return false;
    }
    if (damp_max.getType() != XmlRpcValue::TypeDouble)
    {
	ROS_ERROR_STREAM("Malformed " + param_name + " specification (namespace: " + param_name + ").");
	return false;
    }
    param_name = "kinematics/epsilon";
    if (!controller_nh.getParam(param_name, epsilon))
    {
	ROS_ERROR_STREAM("No epsilon given (expected namespace: " + param_name + ").");
	return false;
    }
    if (epsilon.getType() != XmlRpcValue::TypeDouble)
    {
	ROS_ERROR_STREAM("Malformed " + param_name + " specification (namespace: " + param_name + ").");
	return false;
    }
    param_name = "kinematics/root";
    if (!controller_nh.getParam(param_name, root))
    {
	ROS_ERROR_STREAM("No root given (expected namespace: " + param_name + ").");
	return false;
    }
    if (root.getType() != XmlRpcValue::TypeString)
    {
	ROS_ERROR_STREAM("Malformed " + param_name + " specification (namespace: " + param_name + ").");
	return false;
    }
    param_name = "kinematics/end_effector";
    if (!controller_nh.getParam(param_name, end_effector))
    {
	ROS_ERROR_STREAM("No end_effector given (expected namespace: " + param_name + ").");
	return false;
    }
    if (end_effector.getType() != XmlRpcValue::TypeString)
    {
	ROS_ERROR_STREAM("Malformed " + param_name + " specification (namespace: " + param_name + ").");
	return false;
    }
    param_name = "kinematics/orientation";
    if (!controller_nh.getParam(param_name, orientation))
    {
        ROS_ERROR_STREAM("No orientation given (expected namespace: " + param_name + ").");
        return false;
    }
    if (orientation.getType() != XmlRpcValue::TypeBoolean)
    {
        ROS_ERROR_STREAM("Malformed " + param_name + " specification (namespace: " + param_name + ").");
        return false;
    }

    // Controller sample time
    //dt_ = 1/static_cast<double>(RT_TASK_FREQUENCY);
    
    // KDL kinematics construction
    //std::string root_name = "T0"; //FIXME
    //std::string end_effector_name = "fixed_right_wrist";
    try
    {
	    kin_ = new KDLKinematics (root,end_effector,damp_max,epsilon);
    }
    catch(const std::runtime_error& e)
    {
	    ROS_ERROR_STREAM("Failed to create kdl kinematics: " << e.what());
	    return false;
    }
    
    // Set the kinematic mask
    use_orientation_ = orientation;
    if(use_orientation_)
        kin_->setMask("1,1,1,1,1,1"); // xyz rpy
    else
        kin_->setMask("1,1,1,0,0,0"); // xyz
        
    cart_size_ = kin_->getCartSize();
    Nodf_kin_ = kin_->getNdof();

    // Resize
    torques_cmd_.resize(Nodf_kin_);
    joint_pos_status_.resize(Nodf_kin_);
    joint_vel_status_.resize(Nodf_kin_);
    cart_pose_status_.resize(cart_size_);
    cart_pose_with_quat_status_.resize(7); // NOTE We assume is always xyz + quaternion
    cart_vel_status_.resize(cart_size_);
    f_vm_.resize(cart_size_);
    jacobian_.resize(cart_size_,Nodf_kin_);
    jacobian_t_.resize(Nodf_kin_,cart_size_); 
    
    // Retrain the joint handles
    joints_.resize(Nodf_kin_);
    std::string joint_name;
    for (int i = 0; i < Nodf_kin_; ++i)
    {
      joint_name = kin_->getJointName(i);
      
      // Get a joint handle
      try
      {
	  joints_[i] = hw->getHandle(joint_name);

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
  
    for (int i = 0; i<Nodf_kin_; i++)
    {
      joint_pos_status_(i) = joints_[i].getPosition();
      joint_vel_status_(i) = joints_[i].getVelocity();
    }
  
    kin_->ComputeJac(joint_pos_status_,jacobian_);
    jacobian_t_= jacobian_.transpose();
    kin_->ComputeFk(joint_pos_status_,cart_pose_status_);
    kin_->ComputeFkDot(joint_pos_status_,joint_vel_status_,cart_vel_status_);
    
    if(use_orientation_)
    {
        // Convert RPY Meka to Quaternion
        Eigen::AngleAxisd rollAngle(cart_pose_status_(5), Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(cart_pose_status_(4), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(cart_pose_status_(3), Eigen::Vector3d::UnitX());
        //Eigen::Quaternion<double> qcur = rollAngle  * pitchAngle *  yawAngle;
        //Eigen::Quaternion<double> qcur = pitchAngle * yawAngle * rollAngle;
        //Eigen::Quaternion<double> qcur = yawAngle * pitchAngle  * rollAngle;
        quat_status_ = rollAngle * yawAngle  * pitchAngle;
        cart_pose_with_quat_status_ << cart_pose_status_.segment<3>(0), quat_status_.w(), quat_status_.x(), quat_status_.y(), quat_status_.z();
        
        // Update the virtual mechanisms
        mechanism_manager_.Update(cart_pose_with_quat_status_,cart_vel_status_.segment<3>(0),period.toSec(),f_vm_);
    }
    else
    {
        // Update the virtual mechanisms
        mechanism_manager_.Update(cart_pose_status_,cart_vel_status_,period.toSec(),f_vm_);
    }
    
    torques_cmd_.noalias() = jacobian_t_ * f_vm_;
    
    // HACK to have fixed wrist on the meka
    //if(use_orientation_)
    //{
        for (int i = 0; i<Nodf_kin_; i++)
            joints_[i].setCommand(torques_cmd_(i));
    //}
    //else
    //{
    //    for (int i = 0; i<4; i++)
    //        joints_[i].setCommand(torques_cmd_(i));
    //    for (int i = 4; i<Nodf_kin_; i++)
    //        joints_[i].setCommand(0.0);
    //}

}







/*PLUGINLIB_DECLARE_CLASS(vf_controller,
                        VFController,
                        vf_controller::VFController,
                        controller_interface::ControllerBase)*/

//PLUGINLIB_EXPORT_CLASS(controller::Controller, controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(vf_controller::VFController, controller_interface::ControllerBase);
