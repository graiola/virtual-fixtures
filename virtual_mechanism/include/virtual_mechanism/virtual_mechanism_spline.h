#ifndef VIRTUAL_MECHANISM_SPLINE_H
#define VIRTUAL_MECHANISM_SPLINE_H

////////// VirtualMechanismInterface
#include <virtual_mechanism/virtual_mechanism_interface.h>

////////// BOOST
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

////////// Toolbox
#include "toolbox/spline/spline.h"

namespace virtual_mechanism_spline
{ 

template <class VM_t>  
class VirtualMechanismSpline: public VM_t
{
	public:

      VirtualMechanismSpline(int state_dim, double K, double B, double Kf, double Bf, double fade_gain, const std::string file_path);
	  
      virtual double getDistance(const Eigen::VectorXd& pos);
      virtual void setWeightedDist(const bool activate); // FIXME: Fake function
      virtual double getGaussian(const Eigen::VectorXd& pos);
      void ComputeStateGivenPhase(const double phase_in, Eigen::VectorXd& state_out);
      //void ComputeStateGivenPhase(const double abscisse_in, Eigen::VectorXd& state_out, Eigen::VectorXd& state_out_dot, double& phase_out, double& phase_out_dot);
	  
	protected:

      void CreateSplineFromTxt(const std::string file_path);
	  
      virtual void UpdateJacobian();
      virtual void UpdateState();
      virtual void UpdateStateDot();
      virtual void ComputeInitialState();
      virtual void ComputeFinalState();

       std::vector<tk::spline > splines_xyz_;
       tk::spline spline_phase_;
       tk::spline spline_phase_inv_;

       double z_;
       double z_dot_;
       double z_dot_ref_;

       Eigen::MatrixXd Jz_;
       Eigen::VectorXd err_;


};

}

#endif



