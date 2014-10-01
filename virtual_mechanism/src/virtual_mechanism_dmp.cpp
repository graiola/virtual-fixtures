#include "virtual_mechanism/virtual_mechanism_dmp.h"

using namespace std;
using namespace ros;
using namespace Eigen;
using namespace tool_box;
using namespace virtual_mechanism_interface;
using namespace DmpBbo;

namespace virtual_mechanism_dmp 
{

VirtualMechanismDmp::VirtualMechanismDmp(int state_dim, boost::shared_ptr<dmp_t> dmp_shr_ptr): VirtualMechanismInterface(state_dim) // FIXME
{
  assert(!dmp_shr_ptr_);
  assert(dmp_shr_ptr->dim_orig() ==  state_dim);
  assert(dmp_shr_ptr->dim() > 0);
  // Check if the dmp is trained
  //assert(dmp_shr_ptr->isTrained());
  
  dmp_shr_ptr_ = dmp_shr_ptr;
  
  // Resize the dmp's state vectors
  dmp_state_dim_ = dmp_shr_ptr_->dim();
  dmp_state_status_.resize(dmp_state_dim_);
  dmp_state_command_.resize(dmp_state_dim_);
  dmp_state_command_dot_.resize(dmp_state_dim_);
  
  dmp_state_status_.fill(0.0);
  dmp_state_command_.fill(0.0);
  dmp_state_command_dot_.fill(0.0);
  
}

void VirtualMechanismDmp::Update(const Ref<const VectorXd>& force, const double dt)
{
  
  dmp_shr_ptr_->integrateStep(dt,dmp_state_status_,dmp_state_command_,dmp_state_command_dot_);
  
  VirtualMechanismInterface::Update(force,dt);
  
  // Update the gating system state
  dmp_state_command_ = dmp_state_command_;
  
}

void VirtualMechanismDmp::Update(const Ref<const Eigen::VectorXd>& pos, const Ref<const Eigen::VectorXd>& vel , const double dt)
{
    VirtualMechanismInterface::Update(force_,dt);
}

void VirtualMechanismDmp::UpdateJacobian()
{
  J_ = dmp_state_command_dot_.segment(0,state_dim_);
  J_transp_ = J_.transpose();
}

void VirtualMechanismDmp::UpdateState()
{
  state_ = dmp_state_command_.segment(0,state_dim_);
}


}