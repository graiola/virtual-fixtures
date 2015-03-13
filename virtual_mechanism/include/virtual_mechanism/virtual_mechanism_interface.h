#ifndef VIRTUAL_MECHANISM_INTERFACE_H
#define VIRTUAL_MECHANISM_INTERFACE_H

////////// Toolbox
#include <toolbox/toolbox.h>

////////// ROS
#include <ros/ros.h>

////////// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <boost/concept_check.hpp>
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
	  VirtualMechanismInterface(int state_dim, double K, double B):state_dim_(state_dim),ros_node_ptr_(NULL),phase_(0.0),
	  phase_prev_(0.0),phase_dot_(0.0),K_(K),B_(B),det_(1.0),num_(-1.0),clamp_(1.0),adapt_gains_(false)
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
	      // NOTE We assume that the phase has dim 1x1
	      state_.resize(state_dim);
	      state_dot_.resize(state_dim);
	      torque_.resize(1);
	      force_.resize(state_dim);
	      J_.resize(state_dim,1);
	      J_transp_.resize(1,state_dim);
	      JxJt_.resize(1,1); // NOTE It is used to store the multiplication J * J_transp 
	  }
	
	  virtual ~VirtualMechanismInterface()
	   {
	      if(ros_node_ptr_!=NULL)
		delete ros_node_ptr_;
	   }
	  
	  virtual void Update(Eigen::VectorXd& force, const double dt)
	  {
	    assert(dt > 0.0);
	    
	    // Save the previous phase
	    phase_prev_ = phase_;
  
	    // Update the Jacobian and its transpose
	    UpdateJacobian();
	    //J_transp_ = J_.transpose();
	    
	    // Update the phase
	    UpdatePhase(force,dt);
	    
	    // Saturate the phase if exceeds 1 or 0
	    ApplySaturation();
	    
	    // Compute the new state
	    UpdateState();
	    
	    // Compute the new state dot
	    UpdateStateDot();
	  }
	  
	  virtual void ApplySaturation()
	  {
	      // Saturations
	      if(phase_ > 1.0)
	      {
		//LINE_CLAMP(phase_,clamp_,0.9,1,1,0);
		phase_ = 1;
		//phase_dot_ = 0;
	      }
	      else if (phase_ < 0.0)
	      {
		//LINE_CLAMP(phase_,clamp_,0,0.1,0,1);
		phase_ = 0;
		//phase_dot_ = 0;
	      }
	  }
	  
	  inline void Update(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel , const double dt, const double scale = 1.0)
	  {
	      assert(pos.size() == state_dim_);
	      assert(vel.size() == state_dim_);
	    
	      if(adapt_gains_) //FIXME
		AdaptGains(pos,dt);
	      
	      force_ = K_ * (state_ - pos);
	      force_ = force_ - B_ * vel;
	      force_ = scale * force_;
	      //force_ = scale * (K_ * (state_ - pos) - B_ * (vel));
	      Update(force_,dt);
	  }
	  
	  virtual void AdaptGains(const Eigen::VectorXd& pos, const double dt){}
	  inline void setAdaptGains(const bool adapt_gains) {adapt_gains_ = adapt_gains;}
	  
	  inline double getDet() const {return det_;} // For test purpose
	  inline double getTorque() const {return torque_(0,0);} // For test purpose
	  
	  inline double getPhaseDot() const {return phase_dot_;}
	  inline double getPhase() const {return phase_;}
	  inline void getState(Eigen::VectorXd& state) const {assert(state.size() == state_dim_); state = state_;}
	  inline void getStateDot(Eigen::VectorXd& state_dot) const {assert(state_dot.size() == state_dim_); state_dot = state_dot_;}
	  inline double getK() const {return K_;}
	  inline double getB() const {return B_;}
	  inline void setK(const double& K){assert(K > 0.0); K_ = K;}
	  inline void setB(const double& B){assert(B > 0.0); B_ = B;}
	  
          inline void Init()
          {
              // Initialize the attributes
              UpdateJacobian();
              UpdateState();
              UpdateStateDot();
          }
	  
	protected:
	    
	  virtual void UpdateJacobian()=0;
	  virtual void UpdateState()=0;
	  virtual void UpdatePhase(const Eigen::VectorXd& force, const double dt)=0;
	  
	  inline void UpdateStateDot()
	  {
	      state_dot_ = J_ * phase_dot_;
	  }
	  
	  // Ros node
	  tool_box::RosNode* ros_node_ptr_;
	  
	  // States
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
	  double B_;
	  double K_;
	  
	  // Tmp variables
	  double det_;
	  double num_; 
	  
	  // Clamping
	  double clamp_;
	  
	  // To activate the gain adapting
	  bool adapt_gains_;
};
  
class VirtualMechanismInterfaceFirstOrder : public VirtualMechanismInterface
{
	public:
	  //double K = 300, double B = 34.641016,
	  VirtualMechanismInterfaceFirstOrder(int state_dim, double K = 700, double B = 52.91502622129181, double Bf_max = 1, double epsilon = 10):
	  VirtualMechanismInterface(state_dim,K,B)
	  {
            assert(epsilon > 0.1);
            epsilon_ = epsilon;
	    Bd_ = 0.0;
	    Bd_max_ = Bf_max;
	  }

	protected:
	    
	  virtual void UpdateJacobian()=0;
	  virtual void UpdateState()=0;
	  
	  virtual void UpdatePhase(const Eigen::VectorXd& force, const double dt)
	  {
	      JxJt_.noalias() = J_transp_ * J_;
	      
	      // Adapt Bf
	      Bd_ = std::exp(-4/epsilon_*JxJt_(0,0)) * Bd_max_; // NOTE: Since JxJt_ has dim 1x1 the determinant is the only value in it
	      //Bf_ = std::exp(-4/epsilon_*JxJt_.determinant()) * Bf_max_; // NOTE JxJt_.determinant() is always positive! so it's ok
	      
	      det_ = B_ * JxJt_(0,0) + Bd_ * Bd_;
	      
	      torque_.noalias() = J_transp_ * force;
	      
	      // Compute phase dot
	      phase_dot_ = num_/det_ * torque_(0,0);
	      
	      // Compute the new phase
	      phase_ = phase_dot_ * dt + phase_prev_; // FIXME Switch to RungeKutta if possible
	  }
	  
	protected:
	  
	  double Bd_; // Damp term
	  double Bd_max_; // Max damp term
	  double epsilon_;
	  
};

class VirtualMechanismInterfaceSecondOrder : public VirtualMechanismInterface
{
	public:
	  //double K = 300, double B = 34.641016, double K = 700, double B = 52.91502622129181, 900 60, 800 56.568542494923804
	  VirtualMechanismInterfaceSecondOrder(int state_dim, double K = 700, double B = 52.91502622129181, double Kf = 20, double Bf = 8.94427190999916):
	  VirtualMechanismInterface(state_dim,K,B)
	  {
	      assert(Kf > 0.0);
	      assert(Bf > 0.0);

	      Kf_ = Kf;
	      Bf_ = Bf;
	      phase_dot_prev_ = 0.0;
	      phase_ddot_ = 0.0;
	      active_ = false;
	      
	      // Resize the attributes
	      phase_state_dot_.resize(2); //phase_dot and phase_ddot
	      phase_state_.resize(2); //phase_ and phase_dot
	      phase_state_integrated_.resize(2);

	      k1_.resize(2);
	      k2_.resize(2);
	      k3_.resize(2);
	      k4_.resize(2);
	      
	      k1_.fill(0.0);
              k2_.fill(0.0);
	      k3_.fill(0.0);
	      k4_.fill(0.0);
	      
	      fade_ = 0.0;
              
	      adaptive_gain_ptr_ = new tool_box::AdaptiveGain(Kf_,Kf_/2,0.1);
              
	  }
	
	  virtual ~VirtualMechanismInterfaceSecondOrder()
	   {
	      if(adaptive_gain_ptr_!=NULL)
		delete adaptive_gain_ptr_;
	   }
	  
	  using VirtualMechanismInterface::Update; // Use the VirtualMechanismInterface overloaded function
	  
	  virtual void Update(Eigen::VectorXd& force, const double dt)
	  {
	    
	    phase_dot_prev_ = phase_dot_;
	    VirtualMechanismInterface::Update(force,dt);
	    
	  }
	  
	  virtual void ApplySaturation()
	  {
	      // Saturations
	      if(phase_ > 1.0)
	      {
		//LINE_CLAMP(phase_,clamp_,0.9,1,1,0);
		phase_ = 1;
		phase_dot_ = 0;
	      }
	      else if (phase_ < 0.0)
	      {
		//LINE_CLAMP(phase_,clamp_,0,0.1,0,1);
		phase_ = 0;
		phase_dot_ = 0;
	      }
	  }
	  
	  inline double getPhaseDDot() const {return phase_ddot_;}
	  inline double getFade() const {return fade_;} // For test purpose
	  inline void setActive(const bool active) {active_ = active;}
	  
	  
	protected:
	    
	  virtual void UpdateJacobian()=0;
	  virtual void UpdateState()=0;

	  void IntegrateStepRungeKutta(const double& dt, const double& input, const Eigen::VectorXd& phase_state, Eigen::VectorXd& phase_state_integrated)
	  {
	 
	    phase_state_integrated = phase_state;
	    
	    DynSystem(dt,input,phase_state_integrated);
	    k1_ = phase_state_dot_;
	    
	    phase_state_integrated = phase_state_ + 0.5*dt*k1_;
	    DynSystem(dt,input,phase_state_integrated);
	    k2_ = phase_state_dot_;
	    
	    phase_state_integrated = phase_state_ + 0.5*dt*k2_;
	    DynSystem(dt,input,phase_state_integrated);
	    k3_ = phase_state_dot_;
	    
	    phase_state_integrated = phase_state_ + dt*k3_;
	    DynSystem(dt,input,phase_state_integrated);
	    k4_ = phase_state_dot_;
	    
	    phase_state_integrated = phase_state_ + dt*(k1_ + 2.0*(k2_+k3_) + k4_)/6.0;
	  
	  }
	  
	  inline void DynSystem(const double& dt, const double& input, const Eigen::VectorXd& phase_state)
	  {
	     if(active_)
	     {
		fade_ = 10 * (1 - fade_) * dt + fade_;
		//phase_state_dot_(1) = - B_ * JxJt_(0,0) * phase_state(1) - input + fade_ * (- Bf_ * phase_state(1) + Kf_ * (1 - phase_state(0)));
		//phase_ddot_ = - B_ * JxJt_(0,0) * phase_dot_ - torque_(0,0) - Bf_ * phase_dot_ + Kf_ * (1 - phase_);
	     }
	     else
	     {
		fade_ = 10 * (-fade_) * dt + fade_;
		//phase_state_dot_(1) = - B_ * JxJt_(0,0) * phase_state(1) - input + fade_ * (- Bf_ * phase_state(1) + Kf_ * (1 - phase_state(0)));;
		//phase_ddot_ = - B_ * JxJt_(0,0) * phase_dot_ - torque_(0,0);
	     }
	     Kf_ = adaptive_gain_ptr_->ComputeGain((1 - phase_state(0)));
	     phase_state_dot_(1) = 10*( - B_ * JxJt_(0,0) * phase_state(1) - input + fade_ * (- Bf_ * phase_state(1) + Kf_ * (1 - phase_state(0))) );
	     phase_state_dot_(0) = phase_state(1);
	  }
	  
	  virtual void UpdatePhase(const Eigen::VectorXd& force, const double dt)
	  {
	      JxJt_.noalias() = J_transp_ * J_;
	    
	      torque_.noalias() = J_transp_ * force;

	      phase_state_(0) = phase_;
	      phase_state_(1) = phase_dot_;
	      
	      DynSystem(dt,torque_(0),phase_state_);
	      
	      IntegrateStepRungeKutta(dt,torque_(0),phase_state_,phase_state_integrated_);
	      
	      phase_ = phase_state_integrated_(0);
	      phase_dot_ = phase_state_integrated_(1);
	      phase_ddot_ = phase_state_dot_(1);
	      
	      //DynSystem(const Eigen::VectorXd& phase_state, const double& dt, const double& input);
	      
	      /*if(active_)
	        phase_state_dot_(1) = - B_ * JxJt_(0,0) * phase_state(1) - torque_(0,0) - Bf_ * phase_state(1) + Kf_ * (1 - phase_state(0));
		//phase_ddot_ = - B_ * JxJt_(0,0) * phase_dot_ - torque_(0,0) - Bf_ * phase_dot_ + Kf_ * (1 - phase_);
	      else
		phase_state_dot_(1) = - B_ * JxJt_(0,0) * phase_state(1) - torque_(0,0);
		//phase_ddot_ = - B_ * JxJt_(0,0) * phase_dot_ - torque_(0,0);
	      
	      phase_state_dot_(0) = phase_state(1);*/

	      // Compute the new phase
	      // FIXME Switch to RungeKutta  
	      //phase_dot_ = phase_ddot_ * dt + phase_dot_prev_; 
	      //phase_ = phase_dot_ * dt + phase_prev_;
	  }
	  

	  // Switch to auto-completion
	  bool active_;

	  double phase_dot_prev_;
	  double phase_ddot_;

	  Eigen::VectorXd phase_state_;
	  Eigen::VectorXd phase_state_dot_;
	  Eigen::VectorXd phase_state_integrated_;
	  Eigen::VectorXd state_dot_;
 
	  Eigen::VectorXd k1_, k2_, k3_, k4_;

	  // Fade system variables
	  double Bf_;
	  double Kf_;
	  double fade_;
	  tool_box::AdaptiveGain* adaptive_gain_ptr_;

};

}

#endif



