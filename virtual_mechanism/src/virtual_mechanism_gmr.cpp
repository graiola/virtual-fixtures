#include "virtual_mechanism/virtual_mechanism_gmr.h"

using namespace std;
using namespace ros;
using namespace Eigen;
using namespace tool_box;
using namespace virtual_mechanism_interface;
using namespace DmpBbo;

namespace virtual_mechanism_gmr 
{

VirtualMechanismGmr::VirtualMechanismGmr(int state_dim, boost::shared_ptr<fa_t> fa_ptr): VirtualMechanismInterfaceSecondOrder(state_dim) // FIXME
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
  variance_.resize(1,state_dim_);
  covariance_.resize(state_dim_,state_dim_);
  covariance_inv_.resize(state_dim_,state_dim_);
  normal_vector_.resize(state_dim_);
  variance_.fill(1.0);
  covariance_ = variance_.row(0).asDiagonal();
  covariance_inv_.fill(0.0);
  
  fa_ptr_ = fa_ptr;
  
  std_variance_ = 0.0;
  //distance_ = 0.0;
  max_std_variance_ = 0.1; // HACK
  K_max_ = K_;
  K_min_ = 100;
  
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

void VirtualMechanismGmr::AdaptGains(const Ref<const VectorXd>& pos)
{
   fa_input_(0,0) = phase_; // Convert to Eigen Matrix
   fa_ptr_->predictVariance(fa_input_,variance_);
   
   covariance_ = variance_.row(0).asDiagonal();
   
   /*normal_vector_ = (pos-state_)/(pos-state_).norm(); // NOTE it is the error versor
   
   std_variance_ = std::sqrt(normal_vector_.transpose()*covariance_*normal_vector_);
   
   K_ = K_max_ - (K_max_/max_std_variance_) * std_variance_;
   
   if(K_ < K_min_)
     K_ = K_min_;
   
   B_ = 2*std::sqrt(K_);*/
   
   //std::cout<< std_variance_ << std::endl;
   
}

double VirtualMechanismGmr::getDistance(const Ref<const VectorXd>& pos)
{
  
  for (int i = 1; i<state_dim_; i++) // NOTE We assume that is a diagonal matrix
    covariance_inv_(i,i) = 1/(covariance_(i,i)+0.001); 
  
  //distance_ = std::sqrt(pos.transpose()*covariance_inv_*state_);
  
  return  std::sqrt(pos.transpose()*covariance_inv_*state_);
  
}


}