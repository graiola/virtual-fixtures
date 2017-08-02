#include "vf_controller/vf_controller.h"

using namespace vf_controller;
using namespace mechanism_manager;
using namespace std;
using namespace Eigen;


////////// KDLFRAME2VECTOR
//#define KDLFRAME2VECTOR(kdl_frame_in,vector_out) do { for(int i = 0; i<3; i++) vector_out[i] = kdl_frame_in.p(i); kdl_frame_in.M.x_frame_.M.GetQuaternion(vector_out[3],vector_out[4],vector_out[5],vector_out[6]); } while (0)
#define KDLFRAME2VECTOR(kdl_frame_in,vector_out) do { for(int i = 0; i<3; i++) vector_out[i] = kdl_frame_in.p(i); } while (0)

bool VFController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &nh)
{
  
  assert(hw != NULL);

  if(!(KinematicsBaseController<hardware_interface::EffortJointInterface>::init(hw, nh)))
  {
    ROS_ERROR("VFController: Couldn't initilize KinematicsBaseController.");
    return false;
  }

  try
  {
    mechanism_manager_ = new MechanismManager(3); //TODO
  }
  catch(const std::runtime_error& e)
  {
    ROS_ERROR("VFController: Failed to create the MechanismManager: %s",e.what());
    return false;
  }

  torques_cmd_.resize(kdl_chain_.getNrOfJoints());
  joints_msr_.resize(kdl_chain_.getNrOfJoints());
  cg_cmd_.resize(joint_handles_.size());
  J_.resize(kdl_chain_.getNrOfJoints());
  G_.resize(kdl_chain_.getNrOfJoints());
  //M_.resize(kdl_chain_.getNrOfJoints());
  //C_.resize(kdl_chain_.getNrOfJoints());
  x_.resize(3);
  x_dot_.resize(3);
  f_vm_.resize(3);
  f_ext_.resize(3);

  cg_comp_on_ = false; // By default OFF
  interaction_on_ = false;

  rt_frame_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(nh, "cartesian_pose", 4));
  sub_interaction_ = nh.subscribe("interaction", 1, &VFController::interaction, this);

  return true;
}

void VFController::starting(const ros::Time& time)
{

  // get joint positions/velocities
  for(int i=0; i < joint_handles_.size(); i++)
  {
    joints_msr_.q(i) = joint_handles_[i].getPosition();
    joints_msr_.qdot(i) = joint_handles_[i].getVelocity();
    joint_handles_[i].setCommand(0.0);
  }

  //cmd_mode_ = HOLD;

  fk_pos_solver_->JntToCart(joints_msr_.q, x_frame_);
  jnt_to_jac_solver_->JntToJac(joints_msr_.q,J_);

  // Publish (Starting) Pose
  //publish_pose(x_,x_des_);

  ROS_DEBUG("Starting for VFController complete.");

}

void VFController::update(const ros::Time& time, const ros::Duration& period)
{

  // get joint positions/velocities
  for(int i=0; i < joint_handles_.size(); i++)
  {
    joints_msr_.q(i) = joint_handles_[i].getPosition();
    joints_msr_.qdot(i) = joint_handles_[i].getVelocity();
    joint_handles_[i].setCommand(0.0);
  }

  fk_pos_solver_->JntToCart(joints_msr_.q, x_frame_);
  jnt_to_jac_solver_->JntToJac(joints_msr_.q,J_);
  id_solver_->JntToGravity(joints_msr_.q, G_);

  KDLFRAME2VECTOR(x_frame_,x_);

  x_dot_.noalias() = J_.data.block(0,0,3,kdl_chain_.getNrOfJoints()) * joints_msr_.qdot.data; //NOTE: take only the position part

  // Update the virtual mechanisms
  mechanism_manager_->Update(x_,x_dot_,period.toSec(),f_vm_); //TODO

  // External interaction force computed as an impedance
  if(interaction_on_)
  {
    x_err_ = diff(x_frame_,x_ext_);
    for(int i = 0; i < f_ext_.size(); i++)
      f_ext_(i) = 100.0 * x_err_(i) - 1.0 * x_dot_(i); //NOTE now it takes only xyz
  }
  else
  {
    f_ext_.fill(0.0);
  }

  if (Equal(x_frame_.p, x_ext_.p, 0.005))
  {
      //ROS_INFO("On target");
      interaction_on_ = false;
  }

  torques_cmd_.data.noalias() = J_.data.block(0,0,3,kdl_chain_.getNrOfJoints()).transpose() * (f_vm_ + f_ext_); //TODO Check allocations

  for (int i = 0; i<kdl_chain_.getNrOfJoints(); i++)
  {
    if(cg_comp_on_)
      cg_cmd_(i) = G_(i);
    else
      cg_cmd_(i) = 0.0;

    joint_handles_[i].setCommand(torques_cmd_(i)+cg_cmd_(i));
  }

  publish_frame(x_frame_);

}

void VFController::publish_frame(const KDL::Frame& f)
{
if (rt_frame_pub_->trylock()) {
    rt_frame_pub_->msg_.header.stamp = ros::Time::now();
    tf::poseKDLToMsg(f, rt_frame_pub_->msg_.pose);
    rt_frame_pub_->unlockAndPublish();
}

}

void VFController::interaction(const vf_controller::PoseRPY::ConstPtr &msg)
{
  KDL::Frame frame_des;

  switch(msg->id)
  {
  case 0:
    frame_des = KDL::Frame(
          KDL::Rotation::RPY(msg->orientation.roll,
                             msg->orientation.pitch,
                             msg->orientation.yaw),
          KDL::Vector(msg->position.x,
                      msg->position.y,
                      msg->position.z));
    break;

  case 1: // position only
    frame_des = KDL::Frame(
          KDL::Vector(msg->position.x,
                      msg->position.y,
                      msg->position.z));
    break;

  case 2: // orientation only
    frame_des = KDL::Frame(
          KDL::Rotation::RPY(msg->orientation.roll,
                             msg->orientation.pitch,
                             msg->orientation.yaw));
    break;

  default:
    ROS_INFO("Wrong message ID");
    return;
  }

  x_ext_ = frame_des;
  interaction_on_ = true;
}

PLUGINLIB_EXPORT_CLASS(vf_controller::VFController, controller_interface::ControllerBase);
