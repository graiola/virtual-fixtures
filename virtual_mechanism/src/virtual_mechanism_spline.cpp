#include "virtual_mechanism/virtual_mechanism_spline.h"

using namespace std;
using namespace Eigen;
using namespace tool_box;
using namespace virtual_mechanism_interface;
using namespace tk;

namespace virtual_mechanism_spline
{

template <typename VM_t>
bool VirtualMechanismSpline<VM_t>::LoadModelFromFile(const string file_path)
{
    vector<vector<double> > data; // abscissa phase x y z
    ReadTxtFile(file_path.c_str(),data);

    int n_points = data.size();

    vector<double> abscissa(n_points,0.0);
    vector<double> phase(n_points,0.0);
    vector<vector<double> > xyz(VM_t::state_dim_, std::vector<double>(n_points));

    for(int i=0;i<n_points;i++)
    {
        abscissa[i] = data[i][0];
        phase[i] =  data[i][1];
        for(int j=0;j<VM_t::state_dim_;j++)
            xyz[j][i] = data[i][j+2];
    }

    splines_xyz_.resize(VM_t::state_dim_);

    for(int i=0;i<VM_t::state_dim_;i++)
        splines_xyz_[i].set_points(phase,xyz[i]);

    spline_phase_.set_points(abscissa,phase); // set_points(x,y) ----> z = f(s)
    spline_phase_inv_.set_points(phase,abscissa); // set_points(x,y) ----> s = g(z)

    return true;
}

template <typename VM_t>
VirtualMechanismSpline<VM_t>::VirtualMechanismSpline(const string file_path):
    VM_t()
{
    LoadModelFromFile(file_path);

    Jz_.resize(VM_t::state_dim_,1);
    err_.resize(VM_t::state_dim_);
    err_.fill(0.0);

    z_ = 0.0;
    z_dot_ = 0.0;
    z_dot_ref_ = 0.1;
}

template <typename VM_t>
void VirtualMechanismSpline<VM_t>::UpdateJacobian()
{
    z_dot_ref_ = 1.0/VM_t::exec_time_;

    z_dot_ = VM_t::fade_ *  z_dot_ref_ + (1-VM_t::fade_) * spline_phase_.compute_derivate(VM_t::phase_) * VM_t::phase_dot_; // FIXME constant value arbitrary

    if(VM_t::active_)
        z_ = z_dot_ * VM_t::dt_ + z_;
    else
        z_ = spline_phase_(VM_t::phase_); // abscisse (s) -> phase (z)

    VM_t::phase_dot_ref_ = spline_phase_inv_.compute_derivate(z_) * z_dot_ref_;
    VM_t::phase_ddot_ref_ = spline_phase_inv_.compute_second_derivate(z_) * z_dot_ref_;
    VM_t::phase_ref_ = spline_phase_inv_(z_);

    // Saturate z
    if(z_ > 1.0)
      z_ = 1;
    else if (z_ < 0.0)
      z_ = 0;

    for(int i=0;i<VM_t::state_dim_;i++)
    {
        //this->fa_output_(0,i) = splines_xyz_[i](z_);
        Jz_(i,0) = splines_xyz_[i].compute_derivate(z_);
        VM_t::J_transp_(0,i) = Jz_(i,0) * spline_phase_.compute_derivate(VM_t::phase_);
    }

    VM_t::J_ = VM_t::J_transp_.transpose();
}

template<typename VM_t>
void VirtualMechanismSpline<VM_t>::UpdateState()
{
    for(int i=0;i<VM_t::state_dim_;i++)
        VM_t::state_(i) = splines_xyz_[i](z_);
}

template<typename VM_t>
void VirtualMechanismSpline<VM_t>::UpdateStateDot()
{
    VM_t::state_dot_ = this->Jz_ * this->z_dot_; // Keep the velocities of the demonstrations
}

template<class VM_t>
void VirtualMechanismSpline<VM_t>::ComputeStateGivenPhase(const double phase_in, VectorXd& state_out)
{
   assert(phase_in <= 1.0);
   assert(phase_in >= 0.0);
   assert(state_out.size() == VM_t::state_dim_);

   for(int i=0;i<VM_t::state_dim_;i++)
       state_out(i) = splines_xyz_[i](phase_in);
}

/*template<class VM_t>
void VirtualMechanismGmr<VM_t>::ComputeStateGivenPhase(const double abscisse_in, VectorXd& state_out, VectorXd& state_out_dot, double& phase_out, double& phase_out_dot)
{
  assert(phase_in <= 1.0);
  assert(phase_in >= 0.0);
  assert(state_out.size() == VM_t::state_dim_);
  assert(state_out_dot.size() == VM_t::state_dim_);

  phase_out = spline_phase_(abscisse_in);
  phase_out_dot = spline_phase_.compute_derivate(abscisse_in);

  for(int i=0;i<VM_t::state_dim_;i++)
  {
      VM_t::state_out_dot(i) = splines_xyz_[i].compute_derivate(phase_out) * phase_out_dot;
      VM_t::state_(i) = splines_xyz_[i](phase_out);
  }
}*/

template<class VM_t>
void VirtualMechanismSpline<VM_t>::ComputeInitialState()
{
  ComputeStateGivenPhase(0.0,VM_t::initial_state_);
}

template<class VM_t>
void VirtualMechanismSpline<VM_t>::ComputeFinalState()
{
  ComputeStateGivenPhase(1.0,VM_t::final_state_);
}

template <class VM_t>
bool VirtualMechanismSpline<VM_t>::SaveModelToFile(const string file_path)
{
     return true;
}

template<class VM_t>
double VirtualMechanismSpline<VM_t>::getDistance(const VectorXd& pos)
{
  err_ = pos - VM_t::state_;
  return err_.norm();
}

template<class VM_t>
double VirtualMechanismSpline<VM_t>::getScale(const VectorXd& pos, const double convergence_factor)
{
  return  std::exp(-convergence_factor*getDistance(pos));
}

// Explicitly instantiate the templates, and its member definitions
template class VirtualMechanismSpline<VirtualMechanismInterfaceFirstOrder>;
template class VirtualMechanismSpline<VirtualMechanismInterfaceSecondOrder>;
}
