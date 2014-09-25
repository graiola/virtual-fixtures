#include "virtual_mechanism/virtual_mechanism.h"
#include "../include/virtual_mechanism/virtual_mechanism.h"

using namespace std;
using namespace ros;
using namespace Eigen;
using namespace tool_box;

namespace virtual_mechanism {

VirtualMechanism::VirtualMechanism(int state_dim)
{
  
  ros_node_ptr_ = NULL;
  
  assert(state_dim == 2 || state_dim == 3); 
  state_dim_ = state_dim;
  
  // Initialize the ros node
  try
  {
    ros_node_ptr_ = new RosNode("virtual_mechanism");
  }
  catch(std::runtime_error err)
  {
    std::cout<<err.what()<<std::endl;
  }
  
  //Initialize the attributes
  phase_ = 0.0;
  phase_prev_ = 0.0;
  
  torque_.resize(1);
  force_.resize(state_dim);
  state_.resize(state_dim);
  state_dot_.resize(state_dim);
  J_.resize(state_dim,1);
  
  Pf_.resize(state_dim,1);
  Pi_.resize(state_dim,1);
  
  //Pf_.setOnes(); // FIXME
  //Pi_.setZero(); // FIXME
  
  Pi_ << 0.5, -0.5, -0.2;
  
  Pf_ << 0.5, -0.5, -0.2;
  
  J_ = (Pf_ - Pi_); //NOTE For now they are here because J is constant
  J_transp_ = J_.transpose();
  
//   Bf_ = MatrixXd::Identity(state_dim_,state_dim_);
//   B_ = MatrixXd::Identity(state_dim_,state_dim_);
//   K_ = MatrixXd::Identity(state_dim_,state_dim_);
  
  
  Bf_.resize(1,1);
  
//   B_.resize(1,1);
//   K_.resize(1,1);
  
  Bf_ << 0.0;
  
  K_ = 70.0;
  B_ = 2*std::sqrt(K_);
  
//   B_ << 1.0;
//   K_ << 1.0;
  
  det_.resize(1,1);
  det_ << 1.0;
  
  num_.resize(1,1);
  num_ << 1.0;
  
}

 VirtualMechanism::~VirtualMechanism()
 {
  if(ros_node_ptr_!=NULL)
    delete ros_node_ptr_;
 }

void VirtualMechanism::Update(const Ref<const VectorXd>& force, double dt)
{
  
  phase_prev_ = phase_;
  
  //Map<MatrixXd> Bf(Bf_,1,1);
  
  det_ = (B_ * J_transp_ * J_ + Bf_);
  
  torque_ = J_transp_ * force;
  
  phase_dot_ = num_(0,0)/det_(0,0) * torque_(0);
  
  phase_ = phase_dot_ * dt + phase_prev_; //FIXME Switch to RungeKutta
  
  // Saturation
  if(phase_ < 0)
    phase_ = 0;
  else if (phase_ > 1)
    phase_ = 1;
  
  state_ = Pi_ + phase_ * J_; //NOTE In this case the Jacobian is the same as the direct kinematic, and it is constant
  state_dot_ = J_ * phase_dot_;

}

void VirtualMechanism::Update(const Ref<const VectorXd>& pos, const Ref<const VectorXd>& vel , double dt)
{

  force_ = K_ * (state_ - pos) - B_ * (vel);

  Update(force_,dt);
}


void VirtualMechanism::getState(Ref<VectorXd> state)
{
  assert(state.size() == state_dim_);
  state = state_;
}
void VirtualMechanism::getStateDot(Ref<VectorXd> state_dot)
{
  assert(state_dot.size() == state_dim_);
  state_dot = state_dot_;
}

/*double VirtualMechanism::PhaseSystem(double phase, VectorXd force)
{
}
*/

/*void VirtualMechanism::UpdateJacobian()
{
  
}*/

}	