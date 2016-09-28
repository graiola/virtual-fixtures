/**
 * @file   virtual_mechanism_gmr.cpp
 * @brief  Virtual mechanisms implemented using GMM/GMR.
 * @author Gennaro Raiola
 *
 * This file is part of virtual-fixtures, a set of libraries and programs to create
 * and interact with a library of virtual guides.
 * Copyright (C) 2014-2016 Gennaro Raiola, ENSTA-ParisTech
 *
 * virtual-fixtures is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * virtual-fixtures is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with virtual-fixtures.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "virtual_mechanism/virtual_mechanism_gmr.h"

using namespace std;
using namespace Eigen;
using namespace tool_box;
using namespace DmpBbo;
using namespace tk;
using namespace dtw;

namespace virtual_mechanism
{

template<class VM_t>
bool VirtualMechanismGmrNormalized<VM_t>::CreateModelFromData(const MatrixXd& data)
{
    VirtualMechanismGmr<VM_t>::CreateModelFromData(data);
    Normalize();
}

template<class VM_t>
bool VirtualMechanismGmrNormalized<VM_t>::CreateModelFromFile(const std::string file_path)
{
    if(VirtualMechanismGmr<VM_t>::CreateModelFromFile(file_path))
    {
        Normalize();
        return true;
    }
    else
        return false;
}

template <class VM_t>
VirtualMechanismGmrNormalized<VM_t>::VirtualMechanismGmrNormalized(const std::string file_path):
    VirtualMechanismGmrNormalized()
{
    if(CreateModelFromFile(file_path))
        VM_t::Init();
    else
        PRINT_ERROR("Can not create model from file "<< file_path);
}

template <class VM_t>
VirtualMechanismGmrNormalized<VM_t>::VirtualMechanismGmrNormalized(const MatrixXd& data):
    VirtualMechanismGmrNormalized()
{
    CreateModelFromData(data);
    VM_t::Init();
}

template <class VM_t>
VirtualMechanismGmrNormalized<VM_t>::VirtualMechanismGmrNormalized():
    VirtualMechanismGmr<VM_t>()
{
    if(!ReadConfig())
    {
      PRINT_ERROR("VirtualMechanismGmrNormalized: Can not read config file");
    }

    Jz_.resize(VM_t::state_dim_,1);
    loopCnt = 0;
    z_ = 0.0;
    z_dot_ = 0.0;
    z_dot_ref_ = 0.1;
}

template <class VM_t>
VirtualMechanismGmrNormalized<VM_t>::VirtualMechanismGmrNormalized(const fa_t* const fa) : VirtualMechanismGmrNormalized()
{
    assert(fa!=NULL);
    assert(fa->isTrained());
    this->fa_ = dynamic_cast<fa_t*>(fa->clone());
    Normalize();
    VM_t::Init();
}

template<class VM_t>
VirtualMechanismInterface* VirtualMechanismGmrNormalized<VM_t>::Clone()
{
    return new VirtualMechanismGmrNormalized<VM_t>(this->fa_);
}

template <class VM_t>
void VirtualMechanismGmrNormalized<VM_t>::Normalize()
{
    spline_phase_.clear(); // spline::clear()
    spline_phase_inv_.clear(); // spline::clear()
    splines_xyz_.clear(); // std::vector::clear()

    std::vector<double> phase_for_spline(n_points_splines_);
    std::vector<double> abscisse_for_spline(n_points_splines_);

    Eigen::MatrixXd input_phase(n_points_splines_,1);
    Eigen::MatrixXd output_position(n_points_splines_,VM_t::state_dim_);
    //Eigen::MatrixXd output_position_dot(n_points,VM_t::state_dim_);
    Eigen::MatrixXd position_diff(n_points_splines_-1,VM_t::state_dim_);
    input_phase.col(0) = VectorXd::LinSpaced(n_points_splines_, 0.0, 1.0);

    // Get xyz from GMR using a linspaced phase [0,1], preserve the rhythme
    this->fa_->predict(input_phase,output_position);
    //fa_->predictDot(input_phase,output_position,output_position_dot);

    if(use_spline_xyz_)
    {
        vector<vector<double> > xyz(VM_t::state_dim_, vector<double>(n_points_splines_));
        splines_xyz_.resize(VM_t::state_dim_);
        // Copy to std vectors
        for(int i=0;i<n_points_splines_;i++)
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
        for(int i=0;i<n_points_splines_;i++)
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
        abscisse_for_spline[i] = abscisse_for_spline[i]/abscisse_for_spline[n_points_splines_-1];
    }

    //tool_box::WriteTxtFile("abscisse.txt",abscisse_for_spline);

    spline_phase_.set_points(abscisse_for_spline,phase_for_spline); // set_points(x,y) ----> z = f(s)
    spline_phase_inv_.set_points(phase_for_spline,abscisse_for_spline); // set_points(x,y) ----> s = g(z)
}

template<class VM_t>
bool VirtualMechanismGmrNormalized<VM_t>::ReadConfig()
{
    YAML::Node main_node = CreateYamlNodeFromPkgName(ROS_PKG_NAME);
    if (const YAML::Node& curr_node = main_node["gmr_normalized"])
    {
        curr_node["use_spline_xyz"] >> use_spline_xyz_;
        curr_node["n_points_splines"] >> n_points_splines_;
        curr_node["execution_time"] >> exec_time_;
        assert(n_points_splines_ > 2);
        assert(exec_time_ > 0);
        return true;
    }
    else
        return false;
}

template<class VM_t>
void VirtualMechanismGmrNormalized<VM_t>::TrainModel(const MatrixXd& data)
{
    VirtualMechanismGmr<VM_t>::TrainModel(data);
    Normalize();
}

template<class VM_t>
void VirtualMechanismGmrNormalized<VM_t>::AlignUpdateModel(const MatrixXd& data)
{
    VirtualMechanismGmr<VM_t>::AlignUpdateModel(data);
    Normalize();
}

template <class VM_t>
void VirtualMechanismGmrNormalized<VM_t>::UpdateJacobian()
{

  z_dot_ref_ = 1.0/exec_time_;

  z_dot_ = VM_t::fade_ *  z_dot_ref_ + (VM_t::fade_sys_.GetRef()-VM_t::fade_) * spline_phase_.compute_derivate(VM_t::phase_) * VM_t::phase_dot_; // FIXME constant value arbitrary

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
}

template <class VM_t>
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

template<class VM_t>
void VirtualMechanismGmrNormalized<VM_t>::UpdateState()
{
    VM_t::state_ = this->fa_output_.transpose();
}

template<class VM_t>
void VirtualMechanismGmrNormalized<VM_t>::UpdateStateDot()
{
    VM_t::state_dot_ = this->Jz_ * this->z_dot_; // Keep the velocities of the demonstrations
}

template <class VM_t>
bool VirtualMechanismGmr<VM_t>::SaveModelToFile(const string file_path)
{
    const ModelParametersGMR* model_parameters_gmr = static_cast<const ModelParametersGMR*>(fa_->getModelParameters());
    if(model_parameters_gmr->saveGMMToMatrix(file_path, true)) // overwrite = true
        return true;
    else
        return false;
}

template<class VM_t>
VirtualMechanismGmr<VM_t>::VirtualMechanismGmr(): VM_t()
{
    if(!ReadConfig())
    {
      PRINT_ERROR("VirtualMechanismGmr: Can not read config file");
    }

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
    fa_ = NULL;
}

template <class VM_t>
VirtualMechanismGmr<VM_t>::VirtualMechanismGmr(const std::string file_path) : VirtualMechanismGmr()
{
    if(!CreateModelFromFile(file_path))
        PRINT_ERROR("Can not create model from file "<< file_path);

     VM_t::Init();
}

template <class VM_t>
VirtualMechanismGmr<VM_t>::VirtualMechanismGmr(const MatrixXd& data) : VirtualMechanismGmr()
{
    CreateModelFromData(data);

    VM_t::Init();
}

template <class VM_t>
VirtualMechanismGmr<VM_t>::VirtualMechanismGmr(const fa_t* const fa) : VirtualMechanismGmr()
{
    assert(fa!=NULL);
    assert(fa->isTrained());
    fa_ = dynamic_cast<fa_t*>(fa->clone());
    VM_t::Init();
}

template<class VM_t>
VirtualMechanismInterface* VirtualMechanismGmr<VM_t>::Clone()
{
    return new VirtualMechanismGmr<VM_t>(fa_);
}

template<class VM_t>
bool VirtualMechanismGmr<VM_t>::ReadConfig()
{
    YAML::Node main_node = CreateYamlNodeFromPkgName(ROS_PKG_NAME);
    if (const YAML::Node& curr_node = main_node["gmr"])
    {
        curr_node["n_gaussians"] >> n_gaussians_;
        curr_node["use_align"] >> use_align_;
        assert(n_gaussians_ > 0);
        return true;
    }
    else
        return false;
}

template<class VM_t>
bool VirtualMechanismGmr<VM_t>::CreateModelFromData(const MatrixXd& data)
{
    // If the function approximator already exists, re-train, otherwise create a new function approximator
    if(fa_==NULL)
    {
        MetaParametersGMR* meta_parameters_gmr = new MetaParametersGMR(1,n_gaussians_); // input/phase dimension is 1
        fa_ = new fa_t(meta_parameters_gmr);
    }

    if(use_align_ && fa_->isTrained()) // Update only if the model has been already trained!
        AlignUpdateModel(data);
    else
        TrainModel(data);

    assert(fa_->isTrained());
    assert(fa_->getExpectedInputDim() == 1);
    assert(fa_->getExpectedOutputDim() == VM_t::state_dim_);

    return true;
}

template<class VM_t>
bool VirtualMechanismGmr<VM_t>::CreateModelFromFile(const std::string file_path)
{
    ModelParametersGMR* model_parameters_gmr = ModelParametersGMR::loadGMMFromMatrix(file_path);
    if(model_parameters_gmr!=NULL)
    {
        fa_ = new fa_t(model_parameters_gmr);
        assert(fa_->getExpectedInputDim() == 1);
        assert(fa_->getExpectedOutputDim() == VM_t::state_dim_);
        return true;
    }
    else
        return false;
}

template <class VM_t>
VirtualMechanismGmr<VM_t>::~VirtualMechanismGmr()
{
    delete fa_;
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

/*template<class VM_t>
double VirtualMechanismGmr<VM_t>::PolynomScale(const VectorXd& pos, double w) //  0.001 [m]
{
    err_ = pos - VM_t::state_;
    double x = err_.norm()/w;

    if(x <= 1.0)
        return x*x*x*x - 2*x*x + 1;
    else
        return 0.0;
}*/

template<class VM_t>
void VirtualMechanismGmr<VM_t>::UpdateInvCov()
{
  for (int i = 0; i<VM_t::state_dim_; i++) // NOTE We assume that is a diagonal matrix
        covariance_inv_(i,i) = 1/(covariance_(i,i));
        //covariance_inv_(i,i) = 1.0;

  //covariance_inv_ = covariance_.inverse();
}

template<class VM_t>
double VirtualMechanismGmr<VM_t>::ComputeProbability(const VectorXd& pos)
{
  UpdateInvCov();

  err_ = pos - VM_t::state_;

  // NOTE Since the covariance matrix is a diagonal matrix:
  // output = exp(-0.5*err_.transpose()*covariance_inv_*err_);
  // becomes:
  double prob = 0.0;
  double determinant_cov = 1.0;
  for (int i = 0; i<VM_t::state_dim_; i++)
  {
    prob += err_(i)*err_(i)*covariance_inv_(i,i);
    determinant_cov *= covariance_(i,i);
  }

  prob = exp(-0.5*prob);

  //prob_ = exp(-0.5*err_.transpose()*covariance_inv_*err_);
  //determinant_cov = covariance_inv_.determinant();

  // For invertible matrices (which covar apparently was), det(A^-1) = 1/det(A)
  // Hence the 1.0/covariance_inv_.determinant() below
  //  ( (2\pi)^N*|\Sigma| )^(-1/2)

  prob *= pow(pow(2*M_PI,VM_t::state_.size()) * determinant_cov,-0.5);

  return prob;
}

template<class VM_t>
double VirtualMechanismGmr<VM_t>::getScale(const VectorXd& pos, const double convergence_factor)
{
  return  std::exp(-convergence_factor*getDistance(pos));
  //return ComputeProbability(pos);
}

template<class VM_t>
double VirtualMechanismGmr<VM_t>::getDistance(const VectorXd& pos)
{
  err_ = pos - VM_t::state_;
  return err_.norm();
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::TrainModel(const MatrixXd& data)
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
    //phase.col(0) = VectorXd::LinSpaced(pos.rows(), 0.0, 1.0); // Time
    ComputeAbscisse(pos,phase); // Abscisse
  }
  fa_->trainIncremental(phase,pos);
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::AlignUpdateModel(const MatrixXd& data)
{
    int n_points = data.rows();

    MatrixXd pos, phase;
    MatrixXd pos_ref(n_points,VM_t::state_dim_);
    MatrixXd phase_ref(n_points,1);
    phase_ref.col(0) = VectorXd::LinSpaced(n_points, 0.0, 1.0);

    this->fa_->predict(phase_ref,pos_ref);

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
      //phase.col(0) = VectorXd::LinSpaced(pos.rows(), 0.0, 1.0); // Time
      ComputeAbscisse(pos,phase); // Abscisse
    }

    //std::string file_name = "/home/sybot/gennaro_output/phase_before.txt";
    //WriteTxtFile(file_name.c_str(),phase);

    align_phase(phase,phase_ref,pos,pos_ref);

    //file_name = "/home/sybot/gennaro_output/phase_after.txt";
    //WriteTxtFile(file_name.c_str(),phase);

    fa_->trainIncremental(phase,pos);
}

template<class VM_t>
double VirtualMechanismGmr<VM_t>::ComputeResponsability(const MatrixXd& pos)
{
    return fa_->computeResponsability(pos);
}

template<class VM_t>
double VirtualMechanismGmr<VM_t>::GetResponsability()
{
    return fa_->getCachedResponsability();
}

template<class VM_t>
void VirtualMechanismGmr<VM_t>::CreateRecordedRefs()
{
    //int n_points = 1000;
    //VM_t::state_recorded_.resize(n_points,VM_t::state_dim_);
    //VM_t::phase_recorded_ = VectorXd::LinSpaced(n_points, 0.0, 1.0);

    VM_t::state_recorded_.resize(VM_t::n_points_discretization_,VM_t::state_dim_);
    VM_t::phase_recorded_.resize(VM_t::n_points_discretization_,1);
    VM_t::tmp_dists_.resize(VM_t::n_points_discretization_);
    VM_t::phase_recorded_.col(0) = VectorXd::LinSpaced(VM_t::n_points_discretization_, 0.0, 1.0);

    fa_->predict(VM_t::phase_recorded_,VM_t::state_recorded_);
}

// Explicitly instantiate the templates, and its member definitions
template class VirtualMechanismGmr<VirtualMechanismInterfaceFirstOrder>;
template class VirtualMechanismGmr<VirtualMechanismInterfaceSecondOrder>;
template class VirtualMechanismGmrNormalized<VirtualMechanismInterfaceFirstOrder>;
template class VirtualMechanismGmrNormalized<VirtualMechanismInterfaceSecondOrder>;
}
