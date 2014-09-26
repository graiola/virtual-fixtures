#ifndef VIRTUAL_MECHANISM_H
#define VIRTUAL_MECHANISM_H

////////// ToolBox
#include "toolbox/toolbox.h"

////////// ROS
#include <ros/ros.h>

////////// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>

////////// BOOST
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace virtual_mechanism {
	
class VirtualMechanism
{
	public:
	  VirtualMechanism(int state_dim, Eigen::VectorXd Pf, Eigen::VectorXd Pi);
	  
		
	  ~VirtualMechanism();
	  
	  void Update(const Eigen::Ref<const Eigen::VectorXd>& force, double dt);
	  void Update(const Eigen::Ref<const Eigen::VectorXd>& pos, const Eigen::Ref<const Eigen::VectorXd>& vel , double dt);	  
	
	  void getState(Eigen::Ref<Eigen::VectorXd> state);
	  void getStateDot(Eigen::Ref<Eigen::VectorXd> state_dot);
	  
	  inline double getK(){return K_;}
	  inline double getB(){return B_;}
	  inline double getPhase(){return phase_;}
	  inline double getPhaseDot(){return phase_dot_;}
	  
	protected:
	  
	  //bool active_;
	  double phase_;
	  double phase_prev_;
	  double phase_dot_;
	  int state_dim_;
	  Eigen::VectorXd state_;
	  Eigen::VectorXd state_dot_;
	  Eigen::MatrixXd J_;
	  Eigen::MatrixXd J_transp_;
	  Eigen::VectorXd torque_;
	  Eigen::VectorXd force_;

	  // Gains
	  Eigen::MatrixXd Bf_;
	  double B_;
	  double K_;
	  //void UpdateJacobian();
	  tool_box::RosNode* ros_node_ptr_;
	  
	  // Tmp variables
	  Eigen::MatrixXd det_;
	  Eigen::MatrixXd num_;
	  
	  // Define the virtual fixture as a line with two points
	  Eigen::MatrixXd Pf_;
	  Eigen::MatrixXd Pi_;
	  
};

}

#endif



