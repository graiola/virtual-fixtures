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
	 
	  void setWeightedDist(const bool& activate);
	  void getLocalKernel(Eigen::Ref<Eigen::VectorXd> mean_variance) const;
	  
	protected:
	  
	  virtual void UpdateJacobian();
	  virtual void UpdateState();
	  virtual void AdaptGains(const Eigen::Ref<const Eigen::VectorXd>& pos, const double dt);
	  
	  boost::shared_ptr<fa_t> fa_ptr_;

	  Eigen::MatrixXd fa_input_;
	  Eigen::MatrixXd fa_output_;
	  Eigen::MatrixXd fa_output_dot_;
	  Eigen::MatrixXd variance_;
	  Eigen::MatrixXd covariance_;
	  Eigen::MatrixXd covariance_inv_;
	  
	  Eigen::VectorXd normal_vector_;
	  Eigen::VectorXd prev_normal_vector_;
	  
	  double std_variance_;
	  //double distance_;
	  double max_std_variance_;
	  double K_max_;
	  double K_min_;
	  
	  
	  bool use_weighted_dist_;
	  tool_box::MinJerk gain_adapter_;
	  
};

}

#endif



