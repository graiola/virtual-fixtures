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
VirtualMechanismGmr<VM_t>::VirtualMechanismGmr(int expected_gmm_dim, boost::shared_ptr<fa_t> fa_ptr): VM_t(expected_gmm_dim) // FIXME
{
  assert(fa_ptr);
  assert(fa_ptr->isTrained());
  assert(fa_ptr->getExpectedInputDim() == 1);
  assert(fa_ptr->getExpectedOutputDim() ==  expected_gmm_dim);
  
  fa_input_.resize(1,1);
  fa_output_.resize(1,fa_dim_);
  fa_output_dot_.resize(1,fa_dim_);
  variance_.resize(1,fa_dim_);
  covariance_id_.resize(VM_t::position_dim_,VM_t::position_dim_); // Handle only the covariance of the position
  covariance_id_inv_.resize(VM_t::position_dim_,VM_t::position_dim_);
  //normal_vector_.resize(VM_t::state_dim_);
  //prev_normal_vector_.resize(VM_t::state_dim_);
  err_.resize(VM_t::position_dim_);
  
  variance_.fill(1.0);
  covariance_id_ = variance_.row(0).asDiagonal();
  covariance_id_inv_.fill(0.0);
  err_.fill(0.0);
   
  fa_ptr_ = fa_ptr;
  
  prob_ = 0.0;
  determinant_cov_ = 1.0;
  
  //std_variance_ = 0.0;

  //max_std_variance_ = 0.04; // HACK
  //K_max_ = VM_t::K_;
  //K_min_ = 100;
  
  // By default don't use the Mahalanobis distance
  use_weighted_dist_ = false;
  
  // Create the scale adapter
  //gain_adapter_.Create(K_min_,0.0,0.0,K_max_,max_std_variance_,0.0);
  
  // Initialize the state of the virtual mechanism
  VM_t::Init();
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::ComputeStateGivenPhase(const double phase_in, VectorXd& state_out) // Not for rt
{
  assert(phase_in <= 1.0);
  assert(phase_in >= 0.0);
  assert(state_out.size() == VM_t::state_dim_);
  MatrixXd fa_input, fa_output;
  fa_input.resize(1,1);
  fa_output.resize(1,VM_t::fa_dim_);
  fa_input(0,0) = phase_in;
  fa_ptr_->predict(fa_input,fa_output);
  if(VM_t::fa_dim_ == VM_t::state_dim_)
    state_out = fa_output.transpose();
  else
    state_out << fa_output.transpose(), VM_t::orientation_;
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::ComputeInitialState() 
{
  ComputeStateGivenPhase(0.0,VM_t::initial_state_);
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::ComputeFinalState()
{
  ComputeStateGivenPhase(1.0,VM_t::final_state_);
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdateJacobian()
{
  fa_input_(0,0) = VM_t::phase_; // Convert to Eigen Matrix
  fa_ptr_->predictDot(fa_input_,fa_output_,fa_output_dot_,variance_);
  
  
  covariance_id_ = variance_.row(0).segment<3>(0).asDiagonal();
  
  VM_t::J_transp_ = fa_output_dot_.block<1,3>(0,0); // NOTE The output is transposed! Take only the position part
  VM_t::J_ = VM_t::J_transp_.transpose();
  
   //J_ = fa_output_dot_;
   //J_transp_ = fa_output_dot_.transpose();
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdateState()
{
  UpdatePosition();
  UpdateOrientation();
  VM_t::state_ << VM_t::position_, VM_t::orientation_;
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdatePosition()
{
    VM_t::position_ = fa_output_.block<1,3>(0,0).transpose();
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdateOrientation()
{
  if(VM_t::fa_dim_ == 7)
    VM_t::orientation_ = fa_output_.block<1,4>(0,3).transpose();
}

/*template<class VM_t>
void VirtualMechanismGmr<VM_t>::AdaptGains(const VectorXd& pos,  const double dt)
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
   virtual void ComputeInitialState()=0;
	  virtual void ComputeFinalState()=0;
   gain_adapter_.Compute(std_variance_);
   gain_adapter_.GetX();
   
   VM_t::K_ = dt * (100 * (gain_adapter_.GetX() - VM_t::K_ )) + VM_t::K_;
   
   if(VM_t::K_ < K_min_)
     VM_t::K_ = K_min_;
   else if(VM_t::K_ > K_max_)
       VM_t::K_ = K_max_;
       
   //B_ = 2*std::sqrt(K_);
   //std::cout<< std_variance_ << std::endl;
}*/

template<class VM_t>
void VirtualMechanismGmr<VM_t>::getLocalKernel(VectorXd& mean_variance) const
{
  assert(mean_variance.size() == VM_t::position_dim_*2);
  for(int i = 0; i < VM_t::position_dim_; i++)
  {
    mean_variance(i) = VM_t::position_(i);
    mean_variance(i+3) = variance_(i);
  }
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdateInvCov()
{
  for (int i = 0; i<VM_t::position_dim_; i++) // NOTE We assume that is a diagonal matrix
    covariance_id_inv_(i,i) = 1/(covariance_id_(i,i)+0.001); 
}

template<class VM_t>
double VirtualMechanismGmr<VM_t>::getProbability(const VectorXd& pos)
{
  UpdateInvCov();
  
  err_ = pos - VM_t::position_;
  
  // NOTE Since the covariance matrix is a diagonal matrix:
  // output = exp(-0.5*err_.transpose()*covariance_inv_*err_);
  // becomes:
  prob_ = 0.0;
  determinant_cov_ = 1.0;
  for (int i = 0; i<VM_t::position_dim_; i++)
  {
    prob_ += err_(i)*err_(i)*covariance_id_inv_(i,i);
    determinant_cov_ *= covariance_id_inv_(i,i);
  }
  
  prob_ = exp(-0.5*prob_);
  
  // For invertible matrices (which covar apparently was), det(A^-1) = 1/det(A)
  // Hence the 1.0/covariance_inv_.determinant() below
  //  ( (2\pi)^N*|\Sigma| )^(-1/2)

  prob_ *= pow(pow(2*M_PI,VM_t::position_.size())/determinant_cov_,-0.5);
  //std::cout<<prob_<<std::endl;
  return prob_;
  
  /*UpdateInvCov();
   
  double output = exp(-0.5*(pos - VM_t::state_).transpose()*covariance_inv_*(pos - VM_t::state_));
  // For invertible matrices (which covar apparently was), det(A^-1) = 1/det(A)
  // Hence the 1.0/covariance_inv_.determinant() below
  //  ( (2\pi)^N*|\Sigma| )^(-1/2)

  output *= pow(pow(2*M_PI,VM_t::state_.size())/covariance_inv_.determinant(),-0.5);
  //std::cout<<output<<std::endl;
  
  return output;*/
  
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::setWeightedDist(const bool activate)
{
  use_weighted_dist_ = activate;
}

template<class VM_t>
double VirtualMechanismGmr<VM_t>::getDistance(const VectorXd& pos)
{
  err_ = pos - VM_t::position_;
  if(use_weighted_dist_)
  {
    UpdateInvCov();
    prob_ = 0.0;
    for (int i = 0; i<VM_t::position_dim_; i++)
      prob_ += err_(i)*err_(i)*covariance_id_inv_(i,i);
    return std::sqrt(prob_);
  }
  else
    return err_.norm();
}

// Explicitly instantiate the templates, and its member definitions
template class VirtualMechanismGmr<VirtualMechanismInterfaceFirstOrder>;
template class VirtualMechanismGmr<VirtualMechanismInterfaceSecondOrder>;

}