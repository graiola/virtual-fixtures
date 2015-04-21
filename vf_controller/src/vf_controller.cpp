#include "vf_controller/vf_controller.h"

using namespace vf_controller;
using namespace std;
using namespace kdl_kinematics;
using namespace Eigen;
using namespace XmlRpc;

bool VFController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
  
    XmlRpcValue damp_max, epsilon; 
    std::string param_name = "ik/damp_max";
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
    param_name = "ik/epsilon";
    if (!controller_nh.getParam(param_name, epsilon))
    {
	ROS_ERROR_STREAM("No damp_max given (expected namespace: " + param_name + ").");
	return false;
    }
    if (epsilon.getType() != XmlRpcValue::TypeDouble)
    {
	ROS_ERROR_STREAM("Malformed " + param_name + " specification (namespace: " + param_name + ").");
	return false;
    }

    // Controller sample time
    //dt_ = 1/static_cast<double>(RT_TASK_FREQUENCY);
    
    // IK
    std::string root_name = "T0"; //FIXME
    std::string end_effector_name = "fixed_right_wrist";
    try
    {
	    kin_ = new KDLKinematics (root_name,end_effector_name,damp_max,epsilon);
    }
    catch(const std::runtime_error& e)
    {
	    ROS_ERROR_STREAM("Failed to create kdl kinematics: " << e.what());
	    return false;
    }
    // Set the kinematic mask
    kin_->setMask("1,1,1,0,0,0"); // xyz
    cart_size_ = kin_->getCartSize();
    Nodf_kin_ = kin_->getNdof();
    
    // Resize
    //torques_cmd_.resize(Nodf_kin_);
   
    // Resize
    joint_pos_status_.resize(Nodf_kin_);
    cart_pos_status_.resize(cart_size_);
    cart_pos_cmd_.resize(cart_size_);
    cart_vel_status_.resize(cart_size_);
	
    // User velocity, vf and joint vel commands
    jacobian_.resize(cart_size_,Nodf_kin_);
    jacobian_t_.resize(Nodf_kin_,cart_size_); // only pos
    jacobian_t_pinv_.resize(cart_size_,Nodf_kin_); // only pos
    
    svd_vect_.resize(cart_size_);
    svd_.reset(new svd_t(Nodf_kin_,cart_size_)); // It should have the same dimensionality of the problem
	  
    // Inverse kinematics pre-allocations
    svd_->compute(jacobian_t_, ComputeThinU | ComputeThinV); // This is not rt safe! We trigger it here to pre-allocate the internal variables
    matrixU_t_.resize(cart_size_,Nodf_kin_); // Eigen does some tricks with the dimensionalities, check the .h for info
    matrixV_.resize(cart_size_,cart_size_);
    jacobian_t_pinv_tmp_.resize(cart_size_,cart_size_);
    
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
        joint_pos_status_(i) = joints_[i].getCommand();
  
    kin_->ComputeJac(joint_pos_status_,jacobian_);
    
    jacobian_t_= jacobian_.transpose();
    
    // Compute IK
    svd_->compute(jacobian_t_, ComputeThinU | ComputeThinV);
    svd_vect_ = svd_->singularValues();
    damp_max = 0.001;
    epsilon = 0.01;
    for (int i = 0; i < svd_vect_.size(); i++)
    {
	    svd_curr = svd_vect_[i];
	    damp = std::exp(-4/epsilon*svd_vect_[i])*damp_max;
	    svd_vect_[i] = svd_curr/(svd_curr*svd_curr+damp*damp);
    }
    
    matrixU_t_ = svd_->matrixU().transpose();
    matrixV_ = svd_->matrixV();
    
    jacobian_t_pinv_tmp_ = svd_->matrixV() * svd_vect_.asDiagonal();
    jacobian_t_pinv_.noalias() = jacobian_t_pinv_tmp_ * matrixU_t_; // NOTE .noalias() does the trick
    // End Compute IK

    // Robot cart stuff
    kin_->ComputeFk(joint_pos_status_,cart_pos_status_);
    kin_->ComputeFkDot(joint_pos_status_,joint_vel_status_,cart_vel_status_);
    
    // Update the virtual mechanisms
    mechanism_manager_.Update(cart_pos_status_,cart_vel_status_,dt_,f_vm_);
  
    std::cout<<"MAMMT"<<std::endl;
  
}







/*PLUGINLIB_DECLARE_CLASS(vf_controller,
                        VFController,
                        vf_controller::VFController,
                        controller_interface::ControllerBase)*/

//PLUGINLIB_EXPORT_CLASS(controller::Controller, controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(vf_controller::VFController, controller_interface::ControllerBase);
