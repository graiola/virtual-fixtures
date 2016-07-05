#include "virtual_mechanism/virtual_mechanism_gmr.h"

using namespace std;
using namespace Eigen;
using namespace tool_box;
using namespace virtual_mechanism_interface;
using namespace DmpBbo;
using namespace tk;

namespace virtual_mechanism_gmr 
{

template <typename VM_t>
VirtualMechanismGmrNormalized<VM_t>::VirtualMechanismGmrNormalized(int state_dim, vector<double> K, vector<double> B, double Kf, double Bf, double fade_gain, const string file_path):
    VirtualMechanismGmr<VM_t>(state_dim,K,B,Kf,Bf,fade_gain,file_path)
{
    use_spline_xyz_ = true; // FIXME

    const int n_points = 1000; // This is causing some troubles with the stack
    Jz_.resize(VM_t::state_dim_,1);
    std::vector<double> phase_for_spline(n_points);
    std::vector<double> abscisse_for_spline(n_points);

    Eigen::MatrixXd input_phase(n_points,1);
    Eigen::MatrixXd output_position(n_points,VM_t::state_dim_);
    //Eigen::MatrixXd output_position_dot(n_points,VM_t::state_dim_);
    Eigen::MatrixXd position_diff(n_points-1,VM_t::state_dim_);
    input_phase.col(0) = VectorXd::LinSpaced(n_points, 0.0, 1.0);
    // Get xyz from GMR using a linspaced phase [0,1], preserve the rhythme
    this->fa_->predict(input_phase,output_position);
    //fa_->predictDot(input_phase,output_position,output_position_dot);

    if(use_spline_xyz_)
    {
        vector<vector<double> > xyz(VM_t::state_dim_, vector<double>(n_points));
        splines_xyz_.resize(VM_t::state_dim_);
        // Copy to std vectors
        for(int i=0;i<n_points;i++)
        {
            phase_for_spline[i] = input_phase(i,0);
            for(int j=0;j<VM_t::state_dim_;j++)
                xyz[j][i] = output_position(i,j);
        }
        for(int i=0;i<VM_t::state_dim_;i++)
        {
            splines_xyz_[i].set_points(phase_for_spline,xyz[i]);
        }
    }
    else
    {
        for(int i=0;i<n_points;i++)
            phase_for_spline[i] = input_phase(i,0);
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

    spline_phase_.set_points(abscisse_for_spline,phase_for_spline); // set_points(x,y) ----> z = f(s)
    spline_phase_inv_.set_points(phase_for_spline,abscisse_for_spline); // set_points(x,y) ----> s = g(z)

    loopCnt = 0;
    z_ = 0.0;
    z_dot_ = 0.0;
    z_dot_ref_ = 0.1;
}

template <typename VM_t>
void VirtualMechanismGmrNormalized<VM_t>::UpdateJacobian()
{

  z_dot_ref_ = 1.0/VM_t::exec_time_;

  z_dot_ = VM_t::fade_ *  z_dot_ref_ + (1-VM_t::fade_) * spline_phase_.compute_derivate(VM_t::phase_) * VM_t::phase_dot_; // FIXME constant value arbitrary

  if(VM_t::active_)
  {
      //z_dot_ = VM_t::fade_ *  z_dot_ref_ + (1-VM_t::fade_) * z_dot_; // FIXME constant value arbitrary
      z_ = z_dot_ * VM_t::dt_ + z_;
      // NORMALIZE ONLY IF NOT ACTIVE!!!!!!!!!!!!!!!!!!
      // NOW IS NORMALIZING ALWAYS
  }
  else
  {
      //z_dot_ = spline_phase_.compute_derivate(VM_t::phase_) * VM_t::phase_dot_;
      z_ = spline_phase_(VM_t::phase_); // abscisse (s) -> phase (z)
  }

  // HACKY THING
  // Compute the phase_dot_ref starting by the constant reference in z_dot
  // Ignore all the structure
  // Just out some stuff
  VM_t::phase_dot_ref_ = spline_phase_inv_.compute_derivate(z_) * z_dot_ref_;
  VM_t::phase_ddot_ref_ = spline_phase_inv_.compute_second_derivate(z_) * z_dot_ref_;
  VM_t::phase_ref_ = spline_phase_inv_(z_);

  // Saturate z
  if(z_ > 1.0)
    z_ = 1;
  else if (z_ < 0.0)
    z_ = 0;

  this->fa_input_(0,0) = z_;
  this->fa_->predictDot(this->fa_input_,this->fa_output_,this->fa_output_dot_,this->variance_); // We need this for the covariance
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
      std::cout << ciccio<< std::endl;
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
      this->fa_->predictDot(fa_input,fa_output,fa_output_dot);
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
    VM_t::state_dot_ = this->Jz_ * this->z_dot_; // Keep the velocities of the demonstrations
}

template <typename VM_t>
VirtualMechanismGmr<VM_t>::~VirtualMechanismGmr()
{
    delete fa_;
}

template <typename VM_t>
bool VirtualMechanismGmr<VM_t>::CreateGmrFromTxt(const string file_path)
{
    ModelParametersGMR* model_parameters_gmr = ModelParametersGMR::loadGMMFromMatrix(file_path);
    if(model_parameters_gmr!=NULL)
    {
        fa_ = new fa_t(model_parameters_gmr);
        delete model_parameters_gmr;
        return true;
    }
    else
        return false;
}

template<class VM_t>
VirtualMechanismGmr<VM_t>::VirtualMechanismGmr(int state_dim, vector<double> K, vector<double> B, double Kf, double Bf, double fade_gain, const MatrixXd& data): VM_t(state_dim,K,B,Kf,Bf,fade_gain)
{
    int n_gaussians = 10; // FIXME: to export
    MetaParametersGMR* meta_parameters_gmr = new MetaParametersGMR(1,n_gaussians); // input/phase dimension is 1
    fa_ = new fa_t(meta_parameters_gmr);
    UpdateGuide(data);
    Init();
}

template<class VM_t>
VirtualMechanismGmr<VM_t>::VirtualMechanismGmr(int state_dim, vector<double> K, vector<double> B, double Kf, double Bf, double fade_gain, const string file_path): VM_t(state_dim,K,B,Kf,Bf,fade_gain)
{
    if(CreateGmrFromTxt(file_path))
        Init();
    else
        throw new invalid_argument("Impossible to load GMM from file.");
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::Init()
{
    assert(fa_->isTrained());
    assert(fa_->getExpectedInputDim() == 1);
    assert(fa_->getExpectedOutputDim() == VM_t::state_dim_);

    fa_input_.resize(1,1);
    fa_output_.resize(1,VM_t::state_dim_);
    fa_output_dot_.resize(1,VM_t::state_dim_);
    variance_.resize(1,VM_t::state_dim_);
    covariance_.resize(VM_t::state_dim_,VM_t::state_dim_);
    covariance_inv_.resize(VM_t::state_dim_,VM_t::state_dim_);
    err_.resize(VM_t::state_dim_);

    variance_.fill(1.0);
    covariance_ = variance_.row(0).asDiagonal();
    covariance_inv_.fill(0.0);
    err_.fill(0.0);
    prob_ = 0.0;
    determinant_cov_ = 1.0;

    // By default don't use the Mahalanobis distance
    use_weighted_dist_ = false;

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
  fa_->predict(fa_input,fa_output);
  state_out = fa_output.transpose();
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

  fa_->predictDot(fa_input_,fa_output_,fa_output_dot_,variance_);

  covariance_ = variance_.row(0).asDiagonal();

  //covariance_ = variance_;
  
  VM_t::J_transp_ = fa_output_dot_; // NOTE The output is transposed!
  VM_t::J_ = VM_t::J_transp_.transpose();

}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdateState()
{
  VM_t::state_ = fa_output_.transpose();
}

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
      //covariance_inv_(i,i) = 1/(covariance_(i,i)); //0.001
      if(use_weighted_dist_)
      {
        covariance_inv_(i,i) = 1/(covariance_(i,i)); //0.001
      }
      else
      {
        covariance_inv_(i,i) = 1.0;
      }

    //covariance_inv_ = covariance_.inverse();

}

template<class VM_t>
double VirtualMechanismGmr<VM_t>::getGaussian(const VectorXd& pos, const double scaling_factor)
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
    determinant_cov_ *= covariance_(i,i);
  }
  
  prob_ = exp(-0.5*scaling_factor*prob_);

  //prob_ = exp(-0.5*err_.transpose()*covariance_inv_*err_);
  //determinant_cov_ = covariance_inv_.determinant();
  
  // For invertible matrices (which covar apparently was), det(A^-1) = 1/det(A)
  // Hence the 1.0/covariance_inv_.determinant() below
  //  ( (2\pi)^N*|\Sigma| )^(-1/2)

  prob_ *= pow(pow(2*M_PI,VM_t::state_.size())/determinant_cov_,-0.5);

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
  return err_.norm();
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdateGuide(const MatrixXd& data)
{
  MatrixXd pos, phase;

  // Extract the phase and the pos
  if(data.cols() == VM_t::state_dim_ + 1) // phase + pos
  {  
    phase = data.col(0);
    pos = data.rightCols(VM_t::state_dim_);
  }
  else // only pos
  {
    pos = data;
    phase.resize(pos.rows(),1);
    phase.col(0) = VectorXd::LinSpaced(pos.rows(), 0.0, 1.0);
  }

  std::cout << "pos" << std::endl;
  std::cout << pos << std::endl;
  //std::cout << "phase" << std::endl;
  //std::cout << phase << std::endl;
  getchar();

  fa_->trainIncremental(phase,pos);
}

// Explicitly instantiate the templates, and its member definitions
template class VirtualMechanismGmr<VirtualMechanismInterfaceFirstOrder>;
template class VirtualMechanismGmr<VirtualMechanismInterfaceSecondOrder>;
template class VirtualMechanismGmrNormalized<VirtualMechanismInterfaceFirstOrder>;
template class VirtualMechanismGmrNormalized<VirtualMechanismInterfaceSecondOrder>;
}
