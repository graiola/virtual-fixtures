#ifndef VIRTUAL_MECHANISM_GMR_H
#define VIRTUAL_MECHANISM_GMR_H

////////// VirtualMechanismInterface
#include <virtual_mechanism/virtual_mechanism_interface.h>

////////// Function Approximator
#include <functionapproximators/FunctionApproximatorGMR.hpp>
#include <functionapproximators/MetaParametersGMR.hpp>

////////// BOOST
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace virtual_mechanism_gmr
{
  
  typedef DmpBbo::FunctionApproximatorGMR fa_t;
  
class VirtualMechanismGmr: public virtual_mechanism_interface::VirtualMechanismInterfaceSecondOrder
{
	public:
	  
	  VirtualMechanismGmr(int state_dim, boost::shared_ptr<fa_t> fa_ptr);

	  //void Update(const Eigen::Ref<const Eigen::VectorXd>& force, const double dt);
	  //void Update(const Eigen::Ref<const Eigen::VectorXd>& pos, const Eigen::Ref<const Eigen::VectorXd>& vel , const double dt);
	  
	  double getDistance(const Eigen::Ref<const Eigen::VectorXd>& pos);
	  
	protected:
	  
	  virtual void UpdateJacobian();
	  virtual void UpdateState();

	  boost::shared_ptr<fa_t> fa_ptr_;

	  Eigen::MatrixXd fa_input_;
	  Eigen::MatrixXd fa_output_;
	  Eigen::MatrixXd fa_output_dot_;
	  
};

}

#endif



