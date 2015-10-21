#include "vf_controller/vf_controller.h"

using namespace vf_controller;
using namespace std;
using namespace kdl_kinematics;
using namespace Eigen;
using namespace XmlRpc;

bool VFController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
  
    XmlRpcValue damp_max, epsilon, root, end_effector; 
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
	ROS_ERROR_STREAM("No damp_max given (expected namespace: " + param_name + ").");
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
	ROS_ERROR_STREAM("No damp_max given (expected namespace: " + param_name + ").");
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
	ROS_ERROR_STREAM("No damp_max given (expected namespace: " + param_name + ").");
	return false;
    }
    if (end_effector.getType() != XmlRpcValue::TypeString)
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
    kin_->setMask("1,1,1,1,1,1"); // xyz
    cart_size_ = kin_->getCartSize();
    Nodf_kin_ = kin_->getNdof();

    // Resize
    torques_cmd_.resize(Nodf_kin_);
    joint_pos_status_.resize(Nodf_kin_);
    joint_vel_status_.resize(Nodf_kin_);
    cart_pos_status_.resize(cart_size_);
    cart_vel_status_.resize(cart_size_);
    f_vm_.resize(3);
    t_vm_.resize(3);
    t_vm_.fill(0.0);
    ft_vm_.resize(6);
    jacobian_.resize(cart_size_,Nodf_kin_);
    jacobian_t_.resize(Nodf_kin_,cart_size_); 
    
    // Set the fixed orientation for the tooltip
    qref_.reset(new quaternion_t(1.0,0.0,0.0,0.0)); //1.0,0.0,0.0,0.0    0.70710678,0.0,-0.70710678,0.0
    
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
    kin_->ComputeFk(joint_pos_status_,cart_pos_status_);
    kin_->ComputeFkDot(joint_pos_status_,joint_vel_status_,cart_vel_status_);
    
    // Update the virtual mechanisms
    mechanism_manager_.Update(cart_pos_status_.segment<3>(0),cart_vel_status_.segment<3>(0),period.toSec(),f_vm_);

    //Eigen::VectorXd orientation = cart_pos_status_.segment<3>(3);
    //Eigen::VectorXd orientation_ref(3);
    
    Eigen::AngleAxisd rollAngle(cart_pos_status_(5), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(cart_pos_status_(4), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(cart_pos_status_(3), Eigen::Vector3d::UnitX());
    //Eigen::Quaternion<double> qcur = rollAngle  * pitchAngle *  yawAngle;
    //Eigen::Quaternion<double> qcur = pitchAngle * yawAngle * rollAngle;
    //Eigen::Quaternion<double> qcur = yawAngle * pitchAngle  * rollAngle;
    qcur_ = rollAngle * yawAngle  * pitchAngle;
    
    /*Eigen::MatrixXd rotCur(3,3);
    rotCur = qcur.matrix();
    Eigen::MatrixXd rotRef(3,3);
    rotRef = qref.matrix();
    Eigen::MatrixXd rotWrist(3,3);
    rotWrist = rotCur.transpose() * rotRef; //CHECK*/

    t_vm_(0) = 5*(qcur_.w() * qref_->x() - qref_->w() * qcur_.x());
    t_vm_(1) = 5*(qcur_.w() * qref_->y() - qref_->w() * qcur_.y());
    t_vm_(2) = 5*(qcur_.w() * qref_->z() - qref_->w() * qcur_.z());
    
    //Eigen::Matrix3f rotWrist3 = rotWrist.cast<float>();
    //Eigen::VectorXf ea(3);
    //ea  = rotWrist3.eulerAngles(2, 1, 0);
    //orientation_ref(0) = ea(0,0);
    //orientation_ref(1) = ea(1,0);
    //orientation_ref(2) = ea(2,0);

    /*Eigen::Matrix3f rotWrist3 = rotRef.cast<float>();
    Eigen::VectorXf ea(3);
    ea  = rotWrist3.eulerAngles(2, 1, 0);
    orientation_ref(0) = ea(0,0);
    orientation_ref(1) = ea(1,0);
    orientation_ref(2) = ea(2,0);
    
    std::cout<<"***"<<std::endl;
    std::cout<<orientation_ref<<std::endl;*/

    // Original
    //Eigen::VectorXd joints_orientation_cmd(3);
    //joints_orientation_cmd(2) = -std::atan2(rotWrist(1,0),rotWrist(0,0));
    //joints_orientation_cmd(1) = std::atan2(rotWrist(2,0),std::sqrt(std::pow(rotWrist(2,1),2) + std::pow(rotWrist(2,2),2)));
    //joints_orientation_cmd(0) = std::atan2(rotWrist(2,1),rotWrist(2,2));

    //joints_orientation_cmd(2) = -std::atan2(rotWrist(1,0),rotWrist(0,0));
    //joints_orientation_cmd(1) = std::atan2(rotWrist(2,0),std::sqrt(std::pow(rotWrist(2,1),2) + std::pow(rotWrist(2,2),2)));
    //joints_orientation_cmd(0) = std::atan2(rotWrist(2,1),rotWrist(2,2));

    //Eigen::VectorXd joint_orientation = joint_pos_status_.segment<3>(4);

    //t_vm_ = 0.1 * (orientation_ref - orientation);
    //joints_orientation_cmd_ = joints_orientation_dot_ * dt_ + joint_orientation;
    //f_vm_ << 0.0,0.0,0.0;
    
    ft_vm_ << f_vm_, t_vm_;
    
    torques_cmd_.noalias() = jacobian_t_ * ft_vm_;
    
    for (int i = 0; i<Nodf_kin_; i++)
      joints_[i].setCommand(torques_cmd_(i));
}


/*PLUGINLIB_DECLARE_CLASS(vf_controller,
                        VFController,
                        vf_controller::VFController,
                        controller_interface::ControllerBase)*/

//PLUGINLIB_EXPORT_CLASS(controller::Controller, controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(vf_controller::VFController, controller_interface::ControllerBase);
