#ifndef VIRTUAL_MECHANISM_INTERFACE_H
#define VIRTUAL_MECHANISM_INTERFACE_H

////////// ToolBox
#include "toolbox/toolbox.h"

////////// ROS
#include <ros/ros.h>

////////// Eigen
#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/SVD>
//#include <eigen3/Eigen/Geometry>

////////// BOOST
//#include <boost/shared_ptr.hpp>
//#include <boost/make_shared.hpp>

#define LINE_CLAMP(x,y,x1,x2,y1,y2) do { y = (y2-y1)/(x2-x1) * (x-x1) + y1; } while (0)
	
namespace virtual_mechanism_interface 
{
	
class VirtualMechanismInterface
{
	public:
	  //double K = 300, double B = 34.641016,
	  VirtualMechanismInterface(int state_dim, double K = 700, double B = 52.91502622129181, double Bf = 0.0001):state_dim_(state_dim),ros_node_ptr_(NULL),active_(true),phase_(0.0),
	  phase_prev_(0.0),phase_dot_(0.0),K_(K),B_(B),Bf_(Bf),det_(1.0),num_(-1.0),clamp_(1.0)
	  {
	      assert(state_dim_ == 2 || state_dim_ == 3); 
	      assert(K_ > 0.0);
	      assert(B_ > 0.0);
	      
	      // Initialize the ros node
	      try
	      {
		ros_node_ptr_ = new tool_box::RosNode("virtual_mechanism"); // FIXME change here the name
	      }
	      catch(std::runtime_error err)
	      {
		std::cout<<err.what()<<std::endl;
	      }
	      
	      // Initialize/resize the attributes
	      // NOTE We assume that the phase has dim 1
	      state_.resize(state_dim);
	      state_dot_.resize(state_dim);
	      torque_.resize(1);
	      force_.resize(state_dim);
	      J_.resize(state_dim,1);
	      J_transp_.resize(1,state_dim);
	      JxJt_.resize(1,1); // NOTE It is used to store the multiplication J * J_transp

// 	      Bf_ = 0.0;
// 	      K_ = 300.0;
// 	      B_ = 2*std::sqrt(K_);
// 	      det_ = 1.0;
// 	      num_ = -1.0;
	      
	  }
	
	  ~VirtualMechanismInterface()
	   {
	      if(ros_node_ptr_!=NULL)
		delete ros_node_ptr_;
	   }
	  
	  inline void Update(const Eigen::Ref<const Eigen::VectorXd>& force, const double dt)
	  {
	    
	    assert(dt > 0.0);
	    
	    // Save the previous phase
	    phase_prev_ = phase_;
  
	    // Update the Jacobian and its transpose
	    UpdateJacobian();
	    //J_transp_ = J_.transpose();
	    
	    // Update the phase
	    UpdatePhase(force,dt);
	    
	    // Saturation
	    if(phase_ >= 0.9 && phase_ <= 1.0)
	    {
	      
	      LINE_CLAMP(phase_,clamp_,0.9,1,1,0);
	      //phase_ = 0;
	      //phase_dot_ = 0;
	    }
	    else if (phase_ >= 0.0 && phase_ <= 0.1)
	    {
	      
	      LINE_CLAMP(phase_,clamp_,0,0.1,0,1);
	      //phase_ = 1;
	      //phase_dot_ = 0;
	    }
	    else
	      clamp_ = 1.0;
	    
	    
	    phase_dot_ = phase_dot_ * clamp_;
	    
	    // Compute the new state
	    UpdateState();
	    
	    // Compute the new state dot
	    UpdateStateDot();
	   
	  }
	  
	  inline void Update(const Eigen::Ref<const Eigen::VectorXd>& pos, const Eigen::Ref<const Eigen::VectorXd>& vel , const double dt)
	  {
	      assert(pos.size() == state_dim_);
	      assert(vel.size() == state_dim_);
	    
	      force_ = K_ * (state_ - pos) - B_ * (vel);
	      Update(force_,dt);
	  }
	  
	  inline double getPhaseDot() const {return phase_dot_;}
	  inline double getPhase() const {return phase_;}
	  inline void getState(Eigen::Ref<Eigen::VectorXd> state) const {assert(state.size() == state_dim_); state = state_;}
	  inline void getStateDot(Eigen::Ref<Eigen::VectorXd> state_dot) const {assert(state_dot.size() == state_dim_); state_dot = state_dot_;}
	  inline double getK() const {return K_;}
	  inline double getB() const {return B_;}
	  inline void setK(const double& K){assert(K > 0.0); K_ = K;}
	  inline void setB(const double& B){assert(B > 0.0); B_ = B;}
	  
	protected:
	    
	  virtual void UpdateJacobian()=0;
	  virtual void UpdateState()=0;
	  
	  inline void UpdatePhase(const Eigen::Ref<const Eigen::VectorXd>& force, const double dt)
	  {
	      JxJt_ = J_transp_ * J_;
	      det_ = B_ * JxJt_(0,0) + Bf_;
	      
	      torque_ = J_transp_ * force;
	      
	      // Compute phase dot
	      phase_dot_ = num_/det_ * torque_(0,0); // FIXME I don't like that
	      
	      // Compute the new phase
	      phase_ = phase_dot_ * dt + phase_prev_; // FIXME Switch to RungeKutta  
	  }
	  
	  inline void UpdateStateDot()
	  {
	      state_dot_ = J_ * phase_dot_;
	  }
	  
	  // Ros node
	  tool_box::RosNode* ros_node_ptr_;
	  
	  bool active_; // FIXME Not used yet...
	  double phase_;
	  double phase_prev_;
	  double phase_dot_;
	  int state_dim_;
	  Eigen::VectorXd state_;
	  Eigen::VectorXd state_dot_;
	  Eigen::VectorXd torque_;
	  Eigen::VectorXd force_;
	  Eigen::MatrixXd JxJt_;
	  Eigen::MatrixXd J_;
	  Eigen::MatrixXd J_transp_;

	  // Gains
	  double Bf_;
	  double B_;
	  double K_;
	   
	  // Tmp variables
	  double det_;
	  double num_; 
	  
	  double clamp_;
};

}

#endif



