#ifndef VIRTUAL_MECHANISM_LINE_H
#define VIRTUAL_MECHANISM_LINE_H

////////// VirtualMechanismInterface
#include <virtual_mechanism/virtual_mechanism_interface.h>

namespace virtual_mechanism_line 
{
  
class VirtualMechanismLine: public virtual_mechanism_interface::VirtualMechanismInterface
{
	public:
	  
	  VirtualMechanismLine(int state_dim, Eigen::VectorXd Pf, Eigen::VectorXd Pi); //FIXME Export the other arguments
	  
	  virtual void UpdateJacobian();
	  virtual void UpdateState();
	  
	protected:
	   
	   Eigen::VectorXd Pf_;
	   Eigen::VectorXd Pi_;
};

}

#endif



