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
VirtualMechanismGmr<VM_t>::VirtualMechanismGmr(int state_dim, boost::shared_ptr<fa_t> fa_pos_ptr, boost::shared_ptr<fa_t> fa_phase_dot_ptr): VM_t(state_dim) // FIXME
{
  
  assert(fa_pos_ptr);
  assert(fa_phase_dot_ptr);
  
  assert(fa_pos_ptr->isTrained());
  assert(fa_phase_dot_ptr->isTrained());
  
  assert(fa_pos_ptr->getExpectedInputDim() == 1);
  assert(fa_phase_dot_ptr->getExpectedInputDim() == 1);
  
  assert(fa_pos_ptr->getExpectedOutputDim() == state_dim);
  assert(fa_phase_dot_ptr->getExpectedOutputDim() == 1);
  
  fa_input_.resize(1,1);
  fa_pos_output_.resize(1,VM_t::state_dim_);
  fa_phase_dot_output_.resize(1,1);
  fa_pos_output_dot_.resize(1,VM_t::state_dim_);
  variance_pos_.resize(1,VM_t::state_dim_);
  variance_phase_dot_.resize(1,1);
  covariance_pos_.resize(VM_t::state_dim_,VM_t::state_dim_);
  covariance_pos_inv_.resize(VM_t::state_dim_,VM_t::state_dim_);
  //normal_vector_.resize(VM_t::state_dim_);
  //prev_normal_vector_.resize(VM_t::state_dim_);
  err_.resize(VM_t::state_dim_);
  
  variance_pos_.fill(1.0);
  variance_phase_dot_.fill(0.0);
  covariance_pos_ = variance_pos_.row(0).asDiagonal();
  covariance_pos_inv_.fill(0.0);
  //prev_normal_vector_.fill(0.0);
  err_.fill(0.0);
   
  fa_pos_ptr_ = fa_pos_ptr;
  fa_phase_dot_ptr_ = fa_phase_dot_ptr;
  
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
void VirtualMechanismGmr<VM_t>::ComputeStateGivenPhase(const double phase_in, VectorXd& state_out, double& phase_dot_out) // Not for rt
{
  assert(phase_in <= 1.0);
  assert(phase_in >= 0.0);
  assert(state_out.size() == VM_t::state_dim_);
  MatrixXd fa_input, fa_pos_output, fa_phase_dot_output;
  fa_input.resize(1,1);
  fa_pos_output.resize(1,VM_t::state_dim_);
  fa_phase_dot_output.resize(1,1);
  fa_input(0,0) = phase_in;
  fa_pos_ptr_->predict(fa_input,fa_pos_output);
  fa_phase_dot_ptr_->predict(fa_input,fa_phase_dot_output);
  state_out = fa_pos_output.transpose();
  phase_dot_out = fa_phase_dot_output(0,0);
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::ComputeInitialState() 
{
  /*MatrixXd fa_input, fa_output;
  fa_input.resize(1,1);
  fa_output.resize(1,VM_t::state_dim_);
  fa_input(0,0) = 0.0;
  fa_pos_ptr_->predict(fa_input,fa_output);
  initial_state_ = fa_output.transpose();*/
  
  ComputeStateGivenPhase(0.0,VM_t::initial_state_,VM_t::phase_dot_learnt_); // NOTE At the beginning/end VM_t::phase_dot_learnt_ should be zero 
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::ComputeFinalState()
{
  /*MatrixXd fa_input, fa_output;
  fa_input.resize(1,1);
  fa_output.resize(1,VM_t::state_dim_);
  fa_input(0,0) = 1.0;
  fa_pos_ptr_->predict(fa_input,fa_output);
  final_state_ = fa_output.transpose();*/
  
  ComputeStateGivenPhase(1.0,VM_t::final_state_,VM_t::phase_dot_learnt_); // NOTE At the beginning/end VM_t::phase_dot_learnt_ should be zero 
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdateJacobian()
{
  fa_input_(0,0) = VM_t::phase_; // Convert to Eigen Matrix
  fa_pos_ptr_->predictDot(fa_input_,fa_pos_output_,fa_pos_output_dot_,variance_pos_);
  fa_phase_dot_ptr_->predict(fa_input_,fa_phase_dot_output_,variance_phase_dot_); // NOTE For now we dont use variance_phase_dot_
  
  covariance_pos_ = variance_pos_.row(0).asDiagonal();
  
  VM_t::J_transp_ = fa_pos_output_dot_; // NOTE The output is transposed!
  VM_t::J_ = VM_t::J_transp_.transpose();

   //J_ = fa_pos_output_dot_;
   //J_transp_ = fa_pos_output_dot_.transpose();
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdateState()
{
  VM_t::state_ = fa_pos_output_.transpose();
  VM_t::phase_dot_learnt_ = fa_phase_dot_output_(0,0);
  
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
  assert(mean_variance.size() == VM_t::state_dim_*2);
  for(int i = 0; i < VM_t::state_dim_; i++)
  {
    mean_variance(i) = VM_t::state_(i);
    mean_variance(i+3) = variance_pos_(i);
  }
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdateInvCov()
{
  for (int i = 0; i<VM_t::state_dim_; i++) // NOTE We assume that is a diagonal matrix
    covariance_pos_inv_(i,i) = 1/(covariance_pos_(i,i)+0.001); 
}

template<class VM_t>
double VirtualMechanismGmr<VM_t>::getProbability(const VectorXd& pos)
{
  UpdateInvCov();
  
  err_ = pos - VM_t::state_;
  
  // NOTE Since the covariance matrix is a diagonal matrix:
  // output = exp(-0.5*err_.transpose()*covariance_inv_*err_);
  // becomes:
  prob_ = 0.0;
  determinant_cov_ = 1.0;
  for (int i = 0; i<VM_t::state_dim_; i++)
  {
    prob_ += err_(i)*err_(i)*covariance_pos_inv_(i,i);
    determinant_cov_ *= covariance_pos_inv_(i,i);
  }
  
  prob_ = exp(-0.5*prob_);
  
  // For invertible matrices (which covar apparently was), det(A^-1) = 1/det(A)
  // Hence the 1.0/covariance_inv_.determinant() below
  //  ( (2\pi)^N*|\Sigma| )^(-1/2)

  prob_ *= pow(pow(2*M_PI,VM_t::state_.size())/determinant_cov_,-0.5);
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
  err_ = pos - VM_t::state_;
  if(use_weighted_dist_)
  {
    UpdateInvCov();
    prob_ = 0.0;
    for (int i = 0; i<VM_t::state_dim_; i++)
      prob_ += err_(i)*err_(i)*covariance_pos_inv_(i,i);
    return std::sqrt(prob_);
  }
  else
    return err_.norm();

}

// Explicitly instantiate the templates, and its member definitions
template class VirtualMechanismGmr<VirtualMechanismInterfaceFirstOrder>;
template class VirtualMechanismGmr<VirtualMechanismInterfaceSecondOrder>;

}