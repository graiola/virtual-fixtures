#include "virtual_mechanism/virtual_mechanism_line.h"

using namespace std;
using namespace ros;
using namespace Eigen;
using namespace tool_box;
using namespace virtual_mechanism_interface;

namespace virtual_mechanism_line
{

VirtualMechanismLine::VirtualMechanismLine(int state_dim, VectorXd Pf, VectorXd Pi):VirtualMechanismInterface(state_dim) //FIXME, add the other attributes
{
  
  assert(Pf.size() == state_dim_); 
  assert(Pi.size() == state_dim_); 
 
  Pf_ = Pf;
  Pi_ = Pi;
  
  J_ = (Pf_ - Pi_); //NOTE J and J_transp is constant
  J_transp_ = J_.transpose();
  
}

void VirtualMechanismLine::UpdateJacobian()
{
  //NOTE J is constant in respect of the phase so this function is empty
}

void VirtualMechanismLine::UpdateState()
{
  state_ = Pi_ + phase_ * J_; //NOTE In this case the Jacobian is the same as the direct kinematic, and it is constant
}

}