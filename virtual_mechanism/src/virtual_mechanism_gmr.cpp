#include "virtual_mechanism/virtual_mechanism_gmr.h"

using namespace std;
using namespace ros;
using namespace Eigen;
using namespace tool_box;
using namespace virtual_mechanism_interface;
using namespace DmpBbo;

namespace virtual_mechanism_gmr 
{

VirtualMechanismGmr::VirtualMechanismGmr(int state_dim, boost::shared_ptr<fa_t> fa_ptr): VirtualMechanismInterface(state_dim) // FIXME
{
  assert(fa_ptr);
  assert(fa_ptr->isTrained());
  assert(fa_ptr->getExpectedInputDim() == 1);
  
  fa_input_.resize(1,1);
  fa_output_.resize(state_dim_,1);
  fa_output_dot_.resize(state_dim_,1);
  
 
  fa_ptr->predict(fa_input_,fa_output_);
  assert(fa_output_.size() == state_dim_); //FIXME
  

  fa_ptr_ = fa_ptr;
}

void VirtualMechanismGmr::UpdateJacobian()
{
  fa_input_(0,0) = phase_; // Convert to Eigen Matrix
  fa_ptr_->predictDot(fa_input_,fa_output_,fa_output_dot_);
  J_transp_ = fa_output_dot_; // NOTE The output is transposed!
  J_ = J_transp_.transpose();
}

void VirtualMechanismGmr::UpdateState()
{
  //fa_ptr_->predict(fa_input_,state_);
  state_ = fa_output_.transpose();
}


}