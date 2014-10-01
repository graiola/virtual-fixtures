#ifndef VIRTUAL_MECHANISM_DMP_H
#define VIRTUAL_MECHANISM_DMP_H

////////// VirtualMechanismInterface
#include <virtual_mechanism/virtual_mechanism_interface.h>

////////// DMP
#include <dmp/Dmp.hpp>

////////// BOOST
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace virtual_mechanism_dmp 
{
  
  typedef DmpBbo::Dmp dmp_t;
  
class VirtualMechanismDmp: public virtual_mechanism_interface::VirtualMechanismInterface
{
	public:
	  
	  VirtualMechanismDmp(int state_dim, boost::shared_ptr<dmp_t> dmp_shr_ptr);

	  void Update(const Eigen::Ref<const Eigen::VectorXd>& force, const double dt);
	  void Update(const Eigen::Ref<const Eigen::VectorXd>& pos, const Eigen::Ref<const Eigen::VectorXd>& vel , const double dt);
	  
	protected:
	  
	  virtual void UpdateJacobian();
	  virtual void UpdateState();
	  
	  int dmp_state_dim_;
	  boost::shared_ptr<dmp_t> dmp_shr_ptr_;
	  
	  /** Dmp state vectors. */
	  Eigen::VectorXd dmp_state_status_;
	  Eigen::VectorXd dmp_state_command_;
	  Eigen::VectorXd dmp_state_command_dot_; 
};

}

#endif



