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
  
  // Load from txt file
  //ModelParametersGMR* model_parameters_gmr = ModelParametersGMR::loadGMMFromMatrix(file_name);
  //FunctionApproximatorGMR* fa_ptr = new FunctionApproximatorGMR(model_parameters_gmr);
  
  assert(fa_ptr);
  assert(fa_ptr->isTrained());
  assert(fa_ptr->getExpectedInputDim() == 1);
  assert(fa_ptr->getExpectedOutputDim() == state_dim);
  
  fa_input_.resize(1,1);
  fa_output_.resize(1,state_dim_);
  fa_output_dot_.resize(1,state_dim_);
  
  fa_ptr_ = fa_ptr;
}

void VirtualMechanismGmr::UpdateJacobian()
{
  fa_input_(0,0) = phase_; // Convert to Eigen Matrix
  fa_ptr_->predictDot(fa_input_,fa_output_,fa_output_dot_);
  
  J_transp_ = fa_output_dot_; // NOTE The output is transposed!
  J_ = J_transp_.transpose();
   
   //J_ = fa_output_dot_;
   //J_transp_ = fa_output_dot_.transpose();
}

void VirtualMechanismGmr::UpdateState()
{
  
  //fa_ptr_->predict(fa_input_,state_);
  state_ = fa_output_.transpose();
  
  //state_ = fa_output_;
  
}

double VirtualMechanismGmr::getDistance(const Ref<const VectorXd>& pos)
{
 
  return (pos-state_).norm();
  
}


}