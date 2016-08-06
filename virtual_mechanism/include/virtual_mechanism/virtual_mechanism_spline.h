#ifndef VIRTUAL_MECHANISM_SPLINE_H
#define VIRTUAL_MECHANISM_SPLINE_H

////////// VirtualMechanismInterface
#include <virtual_mechanism/virtual_mechanism_interface.h>

////////// Toolbox
#include "toolbox/spline/spline.h"

namespace virtual_mechanism_spline
{ 

template <class VM_t>  
class VirtualMechanismSpline: public VM_t
{
	public:

      VirtualMechanismSpline(const std::string file_path);
	  
      virtual double getDistance(const Eigen::VectorXd& pos);
      virtual double getScale(const Eigen::VectorXd& pos, const double convergence_factor = 1.0);
      virtual bool SaveModelToFile(const std::string file_path);
      void ComputeStateGivenPhase(const double phase_in, Eigen::VectorXd& state_out);
	  
	protected:

      bool LoadModelFromFile(const std::string file_path);
	  
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
