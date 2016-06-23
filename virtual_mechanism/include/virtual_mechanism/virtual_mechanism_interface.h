#ifndef VIRTUAL_MECHANISM_INTERFACE_H
#define VIRTUAL_MECHANISM_INTERFACE_H

////////// Toolbox
#include "toolbox/toolbox.h"

////////// ROS
//#include <ros/ros.h>

////////// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <boost/concept_check.hpp>
//#include <eigen3/Eigen/SVD>
//#include <eigen3/Eigen/Geometry>

////////// BOOST
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#define LINE_CLAMP(x,y,x1,x2,y1,y2) do { y = (y2-y1)/(x2-x1) * (x-x1) + y1; } while (0)
	
namespace virtual_mechanism_interface 
{

    typedef Eigen::Quaternion<double> quaternion_t;
    
class VirtualMechanismInterface
{
	public:
      VirtualMechanismInterface(int state_dim, double K, double B, double Kf, double Bf, double fade_gain):state_dim_(state_dim),update_quaternion_(false),phase_(0.0),
          phase_prev_(0.0),phase_dot_(0.0),phase_dot_ref_(0.0),phase_ddot_ref_(0.0),phase_ref_(0.0),phase_dot_prev_(0.0),phase_ddot_(0.0),scale_(1.0),r_(0.0),p_(0.0),p_dot_integrated_(0.0),exec_time_(10.0),K_(K),B_(B),clamp_(1.0),adapt_gains_(false),Kf_(Kf),Bf_(Bf),fade_gain_(fade_gain),fade_(0.0),active_(false),move_forward_(true),dt_(0.001)
	  {
	      assert(state_dim_ == 2 || state_dim_ == 3); 
	      assert(K_ > 0.0);
	      assert(B_ > 0.0);
	      assert(Kf_ > 0.0);
          assert(Bf_ > 0.0);
          assert(fade_gain_ > 0.0);

	      // Initialize/resize the attributes
	      // NOTE We assume that the phase has dim 1x1
	      state_.resize(state_dim);
	      state_dot_.resize(state_dim);
	      torque_.resize(1);
	      force_.resize(state_dim);
	      final_state_.resize(state_dim);
	      initial_state_.resize(state_dim);
	      J_.resize(state_dim,1);
	      J_transp_.resize(1,state_dim);
	      JxJt_.resize(1,1); // NOTE It is used to store the multiplication J * J_transp
	      
              // Default quaternions
              //q_start_.reset(new Eigen::Quaternion(1.0,0.0,0.0,0.0));
              //q_end_.reset(new Eigen::Quaternion(1.0,0.0,0.0,0.0));
              //quaternion_ << 1.0,0.0,0.0,0.0;
              
          adaptive_gain_ptr_ = new tool_box::AdaptiveGain(K_,K_/100,0.01); //double gain_at_zero, double gain_at_inf = 0.0, double zero_slope_error_value = 0.0
	  }
	
	  virtual ~VirtualMechanismInterface()
	   {
          //if(ros_node_ptr_!=NULL)
            //delete ros_node_ptr_;
	      if(adaptive_gain_ptr_!=NULL)
            delete adaptive_gain_ptr_;
	   }
	  
	  virtual void Update(Eigen::VectorXd& force, const double dt)
	  {
        assert(dt > 0.0);
	    
        dt_ = dt;

	    // Save the previous phase
	    phase_prev_ = phase_;

        // Save the previous phase_dot
        phase_dot_prev_ = phase_dot_;
  
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
            
        // Compute the new quaternion reference
        if (update_quaternion_)
            UpdateQuaternion();
            
	  }
	  
	  virtual void ApplySaturation()
	  {
	      // Saturations
	      if(phase_ > 1.0)
	      {
            //LINE_CLAMP(phase_,clamp_,0.9,1,1,0);
            phase_ = 1.0;
            phase_dot_ = 0.0;
            phase_ddot_ = 0.0;
	      }
	      else if (phase_ < 0.0)
	      {
            //LINE_CLAMP(phase_,clamp_,0,0.1,0,1);
            phase_ = 0.0;
            phase_dot_ = 0.0;
            phase_ddot_ = 0.0;
	      }
	  }
	  
      virtual void Stop()
      {
          phase_dot_ = 0.0;
          phase_ddot_ = 0.0;
      }

      inline void Update(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel , const double dt, const double scale = 1.0)
	  {
	      assert(pos.size() == state_dim_);
	      assert(vel.size() == state_dim_);
	    
          scale_ = scale;

	      if(adapt_gains_) //FIXME
            AdaptGains(pos,dt);
	      

          //K_ = adaptive_gain_ptr_->ComputeGain((state_ - pos).norm());

	      force_ = K_ * (state_ - pos);
	      force_ = force_ - B_ * vel;
          force_ = scale_ * force_;
	      //force_ = scale * (K_ * (state_ - pos) - B_ * (vel));
	      Update(force_,dt);
	  }
	  
      // TO BE MOVED IN ANOTHER SUBCLASS!
      virtual double getDistance(const Eigen::VectorXd& pos)=0;
      virtual void setWeightedDist(const bool activate)=0;
      //virtual void getLocalKernel(Eigen::VectorXd& mean_variance) const=0;
      virtual double getGaussian(const Eigen::VectorXd& pos, const double scaling_factor = 1.0)=0;

	  virtual void AdaptGains(const Eigen::VectorXd& pos, const double dt){}
	  
      virtual void getInitialPos(Eigen::VectorXd& state) const{assert(state.size() == state_dim_); state = initial_state_;}
      virtual void getFinalPos(Eigen::VectorXd& state) const {assert(state.size() == state_dim_); state = final_state_;}
	  
	  inline void setAdaptGains(const bool adapt_gains) {adapt_gains_ = adapt_gains;}
	  inline void setActive(const bool active) {active_ = active;}
      inline void setExecutionTime(const double time) {assert(time > 0.0); exec_time_ = time;}

	  inline void moveBackward() {move_forward_ = false;}
	  inline void moveForward() {move_forward_ = true;}
	  
	  inline double getTorque() const {return torque_(0,0);} // For test purpose
	  inline double getFade() const {return fade_;} // For test purpose
	  
      inline double getPhaseDotDot() const {return phase_ddot_;}
	  inline double getPhaseDot() const {return phase_dot_;}
	  inline double getPhase() const {return phase_;}
      inline double getPhaseRef() const {return phase_ref_;}
      inline double getPhaseDotRef() const {return phase_dot_ref_;}
      inline double getPhaseDotDotRef() const {return phase_ddot_ref_;}
      inline double getR()const {return r_;}
      inline double getPDotIntegrated() const {return p_dot_integrated_;}
	  inline void getState(Eigen::VectorXd& state) const {assert(state.size() == state_dim_); state = state_;}
	  inline void getStateDot(Eigen::VectorXd& state_dot) const {assert(state_dot.size() == state_dim_); state_dot = state_dot_;}
      inline void getJacobian(Eigen::VectorXd& jacobian) const {jacobian = J_;}
	  inline void getQuaternion(Eigen::VectorXd& q) const 
	  {
              assert(q.size() == 4); 
              q(0) = quaternion_->w();
              q(1) = quaternion_->x();
              q(2) = quaternion_->y();
              q(3) = quaternion_->z();
              
          }
          inline double getKf() const {return Kf_;}
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
              ComputeInitialState();
              ComputeFinalState();
          }

          inline void Init(const std::vector<double>& q_start, const std::vector<double>& q_end)
          {
             assert(q_start.size() == 4);
             assert(q_end.size() == 4);

             q_start_.reset(new quaternion_t(q_start[0],q_start[1],q_start[2],q_start[3]));
             q_end_.reset(new quaternion_t(q_end[0],q_end[1],q_end[2],q_end[3]));

             quaternion_.reset(new quaternion_t(q_start[0],q_start[1],q_start[2],q_start[3]));

             update_quaternion_ = true;

             Init();
          }
	  
	protected:
	    
	  virtual void UpdateJacobian()=0;
	  virtual void UpdateState()=0;
	  virtual void UpdatePhase(const Eigen::VectorXd& force, const double dt)=0;
	  virtual void ComputeInitialState()=0;
	  virtual void ComputeFinalState()=0;

      virtual inline void UpdateStateDot()
	  {
	      state_dot_ = J_ * phase_dot_;
	  }
	  
	  inline void UpdateQuaternion()
      {
          *quaternion_ = q_start_->slerp(phase_,*q_end_);
      }
	  
	  // Ros node
      //tool_box::RosNode* ros_node_ptr_;
	  
	  // States
	  double phase_;
	  double phase_prev_;
	  double phase_dot_;
      double phase_dot_ref_;
      double phase_ddot_ref_;
      double phase_ref_;
      double phase_dot_prev_;
      double phase_ddot_;
      double fade_gain_;
      double scale_;

	  int state_dim_;
      bool update_quaternion_;
	  Eigen::VectorXd state_;
	  Eigen::VectorXd state_dot_;
	  Eigen::VectorXd torque_;
	  Eigen::VectorXd force_;
	  Eigen::VectorXd initial_state_;
	  Eigen::VectorXd final_state_;
      boost::shared_ptr<quaternion_t > q_start_;
      boost::shared_ptr<quaternion_t > q_end_;
      boost::shared_ptr<quaternion_t > quaternion_;
	  Eigen::MatrixXd JxJt_;
	  Eigen::MatrixXd J_;
	  Eigen::MatrixXd J_transp_;
      //Eigen::VectorXd J_vector_;


	  // Gains
	  double B_;
	  double K_;
	  
	  // Clamping
	  double clamp_;
	  
	  // To activate the gain adapting
	  bool adapt_gains_;
	  
	  // Auto completion
	  double Kf_;
      double Bf_;
	  double fade_;
	  bool active_;
	  bool move_forward_;
	  tool_box::AdaptiveGain* adaptive_gain_ptr_;

      double dt_;
      double r_;
      double p_;
      double p_dot_integrated_;
      double exec_time_;



};
  
class VirtualMechanismInterfaceFirstOrder : public VirtualMechanismInterface
{
	public:
      //double K = 300, double B = 34.641016, double K = 700, double B = 52.91502622129181,
      VirtualMechanismInterfaceFirstOrder(int state_dim, double K, double B, double Kf = 100, double Bf = 0.1, double fade_gain = 10.0, double Bd_max = 0.0, double epsilon = 10)://double Kf = 1.25
      VirtualMechanismInterface(state_dim,K,B,Kf,Bf,fade_gain)
	  {
        assert(epsilon > 0.1);
        epsilon_ = epsilon;
	    Bd_ = 0.0;
	    Bd_max_ = Bd_max;
	    det_ = 1.0;
        num_ = -1.0;
	  }

	protected:
	    
	  virtual void UpdateJacobian()=0;
	  virtual void UpdateState()=0;
	  virtual void ComputeInitialState()=0;
	  virtual void ComputeFinalState()=0;
	  
	  virtual void UpdatePhase(const Eigen::VectorXd& force, const double dt)
	  {
	      JxJt_.noalias() = J_transp_ * J_;
          /*
	      // Adapt Bf
	      Bd_ = std::exp(-4/epsilon_*JxJt_(0,0)) * Bd_max_; // NOTE: Since JxJt_ has dim 1x1 the determinant is the only value in it
	      //Bf_ = std::exp(-4/epsilon_*JxJt_.determinant()) * Bf_max_; // NOTE JxJt_.determinant() is always positive! so it's ok
          det_ = B_ * JxJt_(0,0) + Bd_ * Bd_;
          */

          det_ = B_ * JxJt_(0,0) + Bf_;

	      torque_.noalias() = J_transp_ * force;
	      
          if(active_) // NOTE: fade is used only when normalized... FIXME
            fade_ = fade_gain_ * (1 - fade_) * dt + fade_;
	      else
            fade_ = fade_gain_ * (-fade_) * dt + fade_;

          phase_dot_ = num_/det_ * torque_(0,0) + fade_ * (Kf_ * (phase_ref_ - phase_) + Bf_ * phase_dot_ref_);

          // Smooth
          //phase_dot_ = fade_ *  phase_dot_ref_ + (1-fade_) * phase_dot_; // FIXME does it make sense to have it here?

	      // Compute the new phase
	      phase_ = phase_dot_ * dt + phase_prev_; // FIXME Switch to RungeKutta if possible

           // Compute phase_ddot
          phase_ddot_ = (phase_dot_ - phase_dot_prev_)/dt;

	  }
	  
	protected:
	  
	  // Tmp variables
	  double det_;
      double num_;
	  
	  double Bd_; // Damp term
	  double Bd_max_; // Max damp term
	  double epsilon_;
	  
};

class VirtualMechanismInterfaceSecondOrder : public VirtualMechanismInterface
{
	public:
	  //double K = 300, double B = 34.641016, double K = 700, double B = 52.91502622129181, 900 60, 800 56.568542494923804
      VirtualMechanismInterfaceSecondOrder(int state_dim, double K, double B, double Kf = 20, double Bf = 8.94427190999916, double fade_gain = 10.0, double inertia = 0.1, double Kr = 100.0, double Kfi = 0.01):
      VirtualMechanismInterface(state_dim,K,B,Kf,Bf,fade_gain)
	  {
	      
	      assert(Bf > 0.0);
          assert(inertia > 0.0);
          assert(Kr > 0.0);
          assert(Kfi >= 0.0);
	      
	      Bf_ = Bf;
          inertia_ = inertia;
          Kr_ = Kr;
          Kfi_ = Kfi;

	      // Resize the attributes
	      phase_state_.resize(2); //phase_ and phase_dot
          phase_state_dot_.resize(2); //phase_dot and phase_ddot
          phase_state_integrated_.resize(2);

          phase_state_.fill(0.0);
          phase_state_dot_.fill(0.0);
          phase_state_integrated_.fill(0.0);

	      k1_.resize(2);
	      k2_.resize(2);
	      k3_.resize(2);
	      k4_.resize(2);
	      
	      k1_.fill(0.0);
          k2_.fill(0.0);
	      k3_.fill(0.0);
	      k4_.fill(0.0);

          control_ = 0.0;
          error_integrated_ = 0.0;

          r_ = 0.0;   
	  }
	
      inline void setInertia(const double inertia) {assert(inertia > 0.0); inertia_ = inertia;}
      inline void setKr(const double Kr) {assert(Kr > 0.0); Kr_ = Kr;}
      inline void setKfi(const double Kfi) {assert(Kfi >= 0.0); Kfi_ = Kfi;}

	protected:
	    
	  virtual void UpdateJacobian()=0;
	  virtual void UpdateState()=0;
	  virtual void ComputeInitialState()=0;
	  virtual void ComputeFinalState()=0;

      void IntegrateStepRungeKutta(const double& dt, const double& input1, const double& input2, const Eigen::VectorXd& phase_state, Eigen::VectorXd& phase_state_integrated)
	  {
	 
	    phase_state_integrated = phase_state;
	    
        DynSystem(dt,input1,input2,phase_state_integrated);
	    k1_ = phase_state_dot_;
	    
	    phase_state_integrated = phase_state_ + 0.5*dt*k1_;
        DynSystem(dt,input1,input2,phase_state_integrated);
	    k2_ = phase_state_dot_;
	    
	    phase_state_integrated = phase_state_ + 0.5*dt*k2_;
        DynSystem(dt,input1,input2,phase_state_integrated);
	    k3_ = phase_state_dot_;
	    
	    phase_state_integrated = phase_state_ + dt*k3_;
        DynSystem(dt,input1,input2,phase_state_integrated);
	    k4_ = phase_state_dot_;
	    
	    phase_state_integrated = phase_state_ + dt*(k1_ + 2.0*(k2_+k3_) + k4_)/6.0;
	  
	  }
	  
      inline void DynSystem(const double& dt, const double& input1, const double& input2, const Eigen::VectorXd& phase_state)
	  {

         //phase_state_dot_(1) = (1/inertia_)*(- B_ * JxJt_(0,0) * phase_state(1) - input); // Old version with damping
         //phase_state_dot_(1) = (1/inertia_)*(- (B_ * JxJt_(0,0)  + F ) * phase_state(1) - input); // Version with friction
         //phase_state_dot_(1) = (1/inertia_)*(-input - 0.1 * phase_state(1)); // double integrator with friction
         //phase_state_dot_(1) = (1/inertia_)*(-(B_ * JxJt_(0,0) + Bf_) * phase_state(1) - input + fade_ *  (Bf_ * phase_dot_ref_ + Kf_ * (phase_ref_ - phase_state(0)))); // Joly
         //phase_state_dot_(1) = (1/inertia_)*(-0.00001 * phase_state(1) - input + fade_ *  (Bf_ * phase_dot_ref_ + Kf_ * (phase_ref_ - phase_state(0))));
         //phase_state_dot_(1) = (1/inertia_)*(- input + fade_ *  (Bf_ * (phase_dot_ref_ -  phase_state(1)) + Kf_ * (phase_ref_ - phase_state(0)))); // Working no friction, pure double integrator
         //phase_state_dot_(1) = (1/inertia_)*(- input - B_ * JxJt_(0,0) * phase_state(1) + fade_ *  (Bf_ * (phase_dot_ref_ -  phase_state(1)) + Kf_ * (phase_ref_ - phase_state(0))));
         //phase_state_dot_(1) = (1/inertia_)*( - input1 - 0.1 * phase_state(1) + input2 ); // FIXME 0.1 is just a little friction to avoid instability

         phase_state_dot_(1) = (1/inertia_)*( - input1 - (1.0 - scale_) * 1.0 * phase_state(1) + input2 ); // dynamic brakes!
         phase_state_dot_(0) = phase_state(1);
         //phase_state_dot_(0) = fade_ *  phase_dot_ref_  + (1-fade_) * phase_state(1);
	  }
	  
	  virtual void UpdatePhase(const Eigen::VectorXd& force, const double dt)
	  {
	      JxJt_.noalias() = J_transp_ * J_;

	      torque_.noalias() = J_transp_ * force;

          phase_state_(0) = phase_;
          phase_state_(1) = phase_dot_;
	        
          if(active_)
          {
             fade_ = fade_gain_ * (1 - fade_) * dt + fade_;
             //p_dot_integrated_ = p_dot_integrated_ + (inertia_ * phase_ddot_) * dt; // with tau
             //p_dot_integrated_ = p_dot_integrated_ + (- inertia_ * phase_dot_ + control_) * dt; // without tau
             error_integrated_ = Kfi_ * (error_integrated_ + (phase_ref_ - phase_) * dt);
          }
          else
          {
             fade_ = fade_gain_ * (-fade_) * dt + fade_;
             //p_dot_integrated_ = p_;
             error_integrated_ = 0.0;
          }

          control_ = fade_ * (Bf_ * (phase_dot_ref_ - phase_dot_) + Kf_ * (phase_ref_ - phase_) + error_integrated_);
	      
          IntegrateStepRungeKutta(dt,torque_(0),control_,phase_state_,phase_state_integrated_);

          DynSystem(dt,torque_(0),control_,phase_state_); // to compute the dots

          phase_ = phase_state_integrated_(0);
	      phase_dot_ = phase_state_integrated_(1);
          phase_ddot_ = phase_state_dot_(1);

          //p_ = inertia_ * phase_dot_;

          //r_ = Kr_ * (p_ - p_dot_integrated_);

          r_ = torque_(0)/J_transp_.norm();
	  }
	  
	  Eigen::VectorXd phase_state_;
	  Eigen::VectorXd phase_state_dot_;
	  Eigen::VectorXd phase_state_integrated_;
	  Eigen::VectorXd state_dot_;
 
	  Eigen::VectorXd k1_, k2_, k3_, k4_;

	  // Fade system variables
	  double Bf_;
      double inertia_;
      double control_;
      double error_integrated_;
      double Kr_;
      double Kfi_;

};

}

#endif



