#include "virtual_mechanism/virtual_mechanism_gmr.h"

using namespace std;
//using namespace ros;
using namespace Eigen;
using namespace tool_box;
using namespace virtual_mechanism_interface;
using namespace DmpBbo;
using namespace tk;

namespace virtual_mechanism_gmr 
{

template <typename VM_t>
VirtualMechanismGmrNormalized<VM_t>::VirtualMechanismGmrNormalized(int state_dim, double K, double B, boost::shared_ptr<fa_t> fa_ptr):
    VirtualMechanismGmr<VM_t>(state_dim,K,B,fa_ptr)
{
    use_spline_xyz_ = true; // FIXME

    int n_points = 1000;
    Jz_.resize(VM_t::state_dim_,1);
    std::vector<double> phase_for_spline(n_points);
    std::vector<double> abscisse_for_spline(n_points);
    std::vector<std::vector<double> > xyz(VM_t::state_dim_, std::vector<double>(n_points));
    Eigen::MatrixXd input_phase(n_points,1);
    Eigen::MatrixXd output_position(n_points,VM_t::state_dim_);
    //Eigen::MatrixXd output_position_dot(n_points,VM_t::state_dim_);
    Eigen::MatrixXd position_diff(n_points-1,VM_t::state_dim_);
    input_phase.col(0) = VectorXd::LinSpaced(n_points, 0.0, 1.0);
    // Get xyz from GMR using a linspaced phase [0,1], preserve the rhythme
    fa_ptr->predict(input_phase,output_position);
    //fa_ptr->predictDot(input_phase,output_position,output_position_dot);

    splines_xyz_.resize(VM_t::state_dim_);

    // Copy to std vectors
    for(int i=0;i<n_points;i++)
    {
        phase_for_spline[i] = input_phase(i);
        for(int j=0;j<VM_t::state_dim_;j++)
            xyz[j][i] = output_position(i,j);
    }

    // Compute the abscisse curviligne
    abscisse_for_spline[0] = 0.0;
    //position_diff.row(0) = output_position.row(0);
    for(int i=0;i<position_diff.rows();i++)
    {
        position_diff.row(i) = output_position.row(i+1) - output_position.row(i);
        abscisse_for_spline[i+1] = position_diff.row(i).norm() + abscisse_for_spline[i];
    }
    // Normalize
    for(int i=0;i<abscisse_for_spline.size();i++)
    {
        abscisse_for_spline[i] = abscisse_for_spline[i]/abscisse_for_spline[n_points-1];
    }

    //tool_box::WriteTxtFile("abscisse.txt",abscisse_for_spline);

    spline_phase_.set_points(abscisse_for_spline,phase_for_spline); // set_points(x,y)

    for(int i=0;i<VM_t::state_dim_;i++)
    {
        splines_xyz_[i].set_points(phase_for_spline,xyz[i]);
    }

    loopCnt = 0;
    z_ = 0.0;
    z_dot_ = 0.0;

}

template <typename VM_t>
void VirtualMechanismGmrNormalized<VM_t>::UpdateJacobian()
{

  if(VM_t::active_)
  {
      z_dot_ = VM_t::fade_ *  0.15 + (1-VM_t::fade_) * z_dot_; // FIXME constant value arbitrary
      z_ = z_dot_ * VM_t::dt_ + z_;
      // NORMALIZE ONLY IF NOT ACTIVE!!!!!!!!!!!!!!!!!!
      // NOW IS NORMALIZING ALWAYS
  }
  else
  {
      z_dot_ = spline_phase_.compute_derivate(VM_t::phase_) * VM_t::phase_dot_;
      z_ = spline_phase_(VM_t::phase_); // abscisse (s) -> phase (z)
  }

  // Saturate z
  if(z_ > 1.0)
    z_ = 1;
  else if (z_ < 0.0)
    z_ = 0;

  this->fa_input_(0,0) = z_;

  this->fa_ptr_->predictDot(this->fa_input_,this->fa_output_,this->fa_output_dot_,this->variance_); // We need this for the covariance
  this->covariance_ = this->variance_.row(0).asDiagonal();

  if(!use_spline_xyz_) // Compute xyz and J(z) using GMR
  {
      Jz_ = this->fa_output_dot_.transpose(); // J(z)
      VM_t::J_transp_ =  this->fa_output_dot_ * spline_phase_.compute_derivate(VM_t::phase_); // J(z) * d(z)/d(s) = J(s)
  }
  else // Compute xyz and J(z) using the spline
  {
      for(int i=0;i<VM_t::state_dim_;i++)
      {
          this->fa_output_(0,i) = splines_xyz_[i](this->fa_input_(0,0));
          Jz_(i,0) = splines_xyz_[i].compute_derivate(this->fa_input_(0,0));
          VM_t::J_transp_(0,i) = Jz_(i,0) * spline_phase_.compute_derivate(VM_t::phase_);
      }
  }
  VM_t::J_ = VM_t::J_transp_.transpose();

  /*if(loopCnt%100==0)
  {
      std::cout << "****************" << std::endl;
      std::cout << "this->fa_output_" << std::endl;
      std::cout << this->fa_output_<< std::endl;
      std::cout << "this->fa_output_" << std::endl;
      std::cout << this->fa_output_dot_<< std::endl;
  }

  loopCnt++;*/
}

template <typename VM_t>
void VirtualMechanismGmrNormalized<VM_t>::ComputeStateGivenPhase(const double abscisse_in, VectorXd& state_out, VectorXd& state_out_dot, double& phase_out, double& phase_out_dot) // Not for rt
{
  assert(abscisse_in <= 1.0);
  assert(abscisse_in >= 0.0);
  assert(state_out.size() == VM_t::state_dim_);
  assert(state_out_dot.size() == VM_t::state_dim_);
  MatrixXd fa_input, fa_output, fa_output_dot;
  fa_input.resize(1,1);
  fa_output.resize(1,VM_t::state_dim_);
  fa_output_dot.resize(1,VM_t::state_dim_);

  fa_input(0,0) = spline_phase_(abscisse_in);
  phase_out_dot = spline_phase_.compute_derivate(abscisse_in);

  if(!use_spline_xyz_)
  {
      this->fa_ptr_->predictDot(fa_input,fa_output,fa_output_dot);
      state_out = fa_output.transpose();
      state_out_dot = fa_output_dot.transpose() * phase_out_dot;
  }
  else
      for(int i=0;i<VM_t::state_dim_;i++)
      {
        state_out(i) =  splines_xyz_[i](fa_input(0,0));
        state_out_dot(i) = splines_xyz_[i].compute_derivate(fa_input(0,0)) * phase_out_dot;
      }

  phase_out = fa_input(0,0);

}

template<typename VM_t>
void VirtualMechanismGmrNormalized<VM_t>::UpdateState()
{
    VM_t::state_ = this->fa_output_.transpose();
}

template<typename VM_t>
void VirtualMechanismGmrNormalized<VM_t>::UpdateStateDot()
{

    /*if(loopCnt%100==0)
    {
        std::cout << "****************" << std::endl;
        std::cout << "this->z_dot_" << std::endl;
        std::cout << this->z_dot_<< std::endl;
    }*/

    VM_t::state_dot_ = this->Jz_ * this->z_dot_; // Keep the velocities of the demonstrations

    //VM_t::state_dot_ = VM_t::J_ * VM_t::phase_dot_;


    /*if(loopCnt%100==0)
    {
        std::cout << "****************" << std::endl;
        std::cout << "VM_t::state_dot_ = VM_t::J_ * VM_t::phase_dot_;" << std::endl;
        std::cout << VM_t::J_ * VM_t::phase_dot_ << std::endl;
        std::cout << "VM_t::state_dot_ = this->J_no_norm_ * this->z_dot_;" << std::endl;
        std::cout << this->J_no_norm_ * this->z_dot_<< std::endl;
    }*/



}

template<class VM_t>
VirtualMechanismGmr<VM_t>::VirtualMechanismGmr(int state_dim, double K, double B, boost::shared_ptr<fa_t> fa_ptr): VM_t(state_dim,K,B) // FIXME
{
  
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
  //normal_vector_.resize(VM_t::state_dim_);
  //prev_normal_vector_.resize(VM_t::state_dim_);
  err_.resize(VM_t::state_dim_);

  variance_.fill(1.0);
  covariance_ = variance_.row(0).asDiagonal();
  covariance_inv_.fill(0.0);
  //prev_normal_vector_.fill(0.0);
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
  fa_output.resize(1,VM_t::state_dim_);
  fa_input(0,0) = phase_in;
  fa_ptr_->predict(fa_input,fa_output);
  state_out = fa_output.transpose();
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::ComputeInitialState() 
{
  /*MatrixXd fa_input, fa_output;
  fa_input.resize(1,1);
  fa_output.resize(1,VM_t::state_dim_);
  fa_input(0,0) = 0.0;
  fa_ptr_->predict(fa_input,fa_output);
  initial_state_ = fa_output.transpose();*/
  
  ComputeStateGivenPhase(0.0,VM_t::initial_state_);
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::ComputeFinalState()
{
  /*MatrixXd fa_input, fa_output;
  fa_input.resize(1,1);
  fa_output.resize(1,VM_t::state_dim_);
  fa_input(0,0) = 1.0;
  fa_ptr_->predict(fa_input,fa_output);
  final_state_ = fa_output.transpose();*/
  
  ComputeStateGivenPhase(1.0,VM_t::final_state_);
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
    mean_variance(i+3) = variance_(i);
  }
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdateInvCov()
{

  for (int i = 0; i<VM_t::state_dim_; i++) // NOTE We assume that is a diagonal matrix
      if(use_weighted_dist_)
        covariance_inv_(i,i) = 1/(covariance_(i,i)+0.001); //0.001
      else
      {
        covariance_inv_(i,i) = 1.0;
      }

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
    prob_ += err_(i)*err_(i)*covariance_inv_(i,i);
    determinant_cov_ *= covariance_inv_(i,i);
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
      prob_ += err_(i)*err_(i)*covariance_inv_(i,i);
    return std::sqrt(prob_);
  }
  else
    return err_.norm();

}

// Explicitly instantiate the templates, and its member definitions
template class VirtualMechanismGmr<VirtualMechanismInterfaceFirstOrder>;
template class VirtualMechanismGmr<VirtualMechanismInterfaceSecondOrder>;
template class VirtualMechanismGmrNormalized<VirtualMechanismInterfaceFirstOrder>;
template class VirtualMechanismGmrNormalized<VirtualMechanismInterfaceSecondOrder>;
}
