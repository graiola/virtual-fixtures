#include "virtual_mechanism/virtual_mechanism_gmr.h"

using namespace std;
using namespace ros;
using namespace Eigen;
using namespace tool_box;
using namespace virtual_mechanism_interface;
using namespace DmpBbo;

namespace virtual_mechanism_gmr 
{

template<class VM_t>
VirtualMechanismGmr<VM_t>::VirtualMechanismGmr(int state_dim, boost::shared_ptr<fa_t> fa_ptr): VM_t(state_dim) // FIXME
{
  
  // Load from txt file
  //ModelParametersGMR* model_parameters_gmr = ModelParametersGMR::loadGMMFromMatrix(file_name);
  //FunctionApproximatorGMR* fa_ptr = new FunctionApproximatorGMR(model_parameters_gmr);
  
  assert(fa_ptr);
  assert(fa_ptr->isTrained());
  assert(fa_ptr->getExpectedInputDim() == 1);
  assert(fa_ptr->getExpectedOutputDim() == state_dim);
  
  fa_input_.resize(1,1);
  fa_output_.resize(1,VM_t::state_dim_);
  fa_output_dot_.resize(1,VM_t::state_dim_);
  variance_.resize(1,VM_t::state_dim_);
  covariance_.resize(VM_t::state_dim_,VM_t::state_dim_);
  covariance_inv_.resize(VM_t::state_dim_,VM_t::state_dim_);
  normal_vector_.resize(VM_t::state_dim_);
  prev_normal_vector_.resize(VM_t::state_dim_);
  variance_.fill(1.0);
  covariance_ = variance_.row(0).asDiagonal();
  covariance_inv_.fill(0.0);
  prev_normal_vector_.fill(0.0);
  
  fa_ptr_ = fa_ptr;
  
  std_variance_ = 0.0;
  //distance_ = 0.0;
  max_std_variance_ = 0.04; // HACK
  K_max_ = VM_t::K_;
  K_min_ = 100;
  
  // By default don't use the Mahalanobis distance
  use_weighted_dist_ = false;
  
  // Create the scale adapter
  gain_adapter_.Create(K_min_,0.0,0.0,K_max_,max_std_variance_,0.0);

}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdateJacobian()
{
  fa_input_(0,0) = VM_t::phase_; // Convert to Eigen Matrix
  fa_ptr_->predictDot(fa_input_,fa_output_,fa_output_dot_,variance_);
  
  covariance_ = variance_.row(0).asDiagonal();
  
  VM_t::J_transp_ = fa_output_dot_; // NOTE The output is transposed!
  VM_t::J_ = VM_t::J_transp_.transpose();

   //J_ = fa_output_dot_;
   //J_transp_ = fa_output_dot_.transpose();
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdateState()
{
  VM_t::state_ = fa_output_.transpose();
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::AdaptGains(const Ref<const VectorXd>& pos,  const double dt)
{
   fa_input_(0,0) = VM_t::phase_; // Convert to Eigen Matrix
   
   //fa_ptr_->predictVariance(fa_input_,variance_);
   //covariance_ = variance_.row(0).asDiagonal();
   
   if((pos-VM_t::state_).norm() <= 0.001)
   {
     std::cout<<"skip"<<std::endl;
     normal_vector_ = prev_normal_vector_;
   }
   else
     normal_vector_ = (pos-VM_t::state_)/(pos-VM_t::state_).norm(); // NOTE it is the error versor
   
   std_variance_ = std::sqrt(normal_vector_.transpose()*covariance_*normal_vector_);
   
   prev_normal_vector_ = normal_vector_;
   
   //K_ = K_max_ - (K_max_/max_std_variance_) * std_variance_;
   //K_ = dt * (100 * ((K_max_ - (K_max_/max_std_variance_) * std_variance_) - K_ )) + K_;
   
   gain_adapter_.Compute(std_variance_);
   gain_adapter_.GetX();
   
   VM_t::K_ = dt * (100 * (gain_adapter_.GetX() - VM_t::K_ )) + VM_t::K_;
   
   if(VM_t::K_ < K_min_)
     VM_t::K_ = K_min_;
   else if(VM_t::K_ > K_max_)
       VM_t::K_ = K_max_;
       
   //B_ = 2*std::sqrt(K_);
   //std::cout<< std_variance_ << std::endl;
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::getLocalKernel(Ref<VectorXd> mean_variance) const
{
  assert(mean_variance.size() == VM_t::state_dim_*2);
  for(int i = 0; i < VM_t::state_dim_; i++)
  {
    mean_variance(i) = VM_t::state_(i);
    mean_variance(i+3) = variance_(i);
  }
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdateInvCov()
{
  for (int i = 0; i<VM_t::state_dim_; i++) // NOTE We assume that is a diagonal matrix
    covariance_inv_(i,i) = 1/(covariance_(i,i)+0.001); 
}

template<class VM_t>
double VirtualMechanismGmr<VM_t>::getProbability(const Ref<const VectorXd>& pos)
{
  UpdateInvCov();
   
  double output = exp(-0.5*(pos - VM_t::state_).transpose()*covariance_inv_*(pos - VM_t::state_));
  // For invertible matrices (which covar apparently was), det(A^-1) = 1/det(A)
  // Hence the 1.0/covariance_inv_.determinant() below
  //  ( (2\pi)^N*|\Sigma| )^(-1/2)

  return output *= pow(pow(2*M_PI,VM_t::state_.size())/covariance_inv_.determinant(),-0.5);
  //return output;
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::setWeightedDist(const bool& activate)
{
  use_weighted_dist_ = activate;
}

template<class VM_t>
double VirtualMechanismGmr<VM_t>::getDistance(const Ref<const VectorXd>& pos)
{
  
  if(use_weighted_dist_)
  {
    UpdateInvCov();
    //for (int i = 1; i<state_dim_; i++) // NOTE We assume that is a diagonal matrix
    //  covariance_inv_(i,i) = 1/(covariance_(i,i)+0.001); 
    return std::sqrt((pos - VM_t::state_).transpose()*covariance_inv_*(pos - VM_t::state_));
  }
  else
    return  (pos - VM_t::state_).norm();

}

// Explicitly instantiate the templates, and its member definitions
template class VirtualMechanismGmr<VirtualMechanismInterfaceFirstOrder>;
template class VirtualMechanismGmr<VirtualMechanismInterfaceSecondOrder>;

}