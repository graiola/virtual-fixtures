/**
 * @file   virtual_mechanism_interface.h
 * @brief  Abstract virtual mechanism interfaces.
 * @author Gennaro Raiola
 *
 * This file is part of virtual-fixtures, a set of libraries and programs to create
 * and interact with a library of virtual guides.
 * Copyright (C) 2014-2016 Gennaro Raiola, ENSTA-ParisTech
 *
 * virtual-fixtures is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * virtual-fixtures is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with virtual-fixtures.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VIRTUAL_MECHANISM_INTERFACE_H
#define VIRTUAL_MECHANISM_INTERFACE_H

////////// ROS
#include <ros/ros.h>

////////// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <boost/concept_check.hpp>

////////// BOOST
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

////////// Toolbox
#include <toolbox/toolbox.h>

////////// Autom
#include "virtual_mechanism/virtual_mechanism_autom.h"

#define LINE_CLAMP(x,y,x1,x2,y1,y2) do { y = (y2-y1)/(x2-x1) * (x-x1) + y1; } while (0)

namespace virtual_mechanism
{
    typedef Eigen::Quaternion<double> quaternion_t;

class VirtualMechanismInterface
{
	public:
      VirtualMechanismInterface():update_quaternion_(false),phase_(0.0),
          phase_prev_(0.0),phase_dot_(0.0),phase_dot_ref_(0.0),
          phase_ddot_ref_(0.0),phase_ref_(0.0),phase_dot_prev_(0.0),
          phase_ddot_(0.0),scale_(1.0),
          fade_(0.0),active_(false),check_activation_(false),dt_(0.001)
	  {

          if(!ReadConfig())
          {
            PRINT_ERROR("VirtualMechanismInterface: Can not read config file");
          }

	      // Initialize/resize the attributes
          // NOTE We assume that the phase has dim 1x1
          state_.resize(state_dim_);
          state_dot_.resize(state_dim_);
          displacement_.resize(state_dim_);
	      torque_.resize(1);
          force_.resize(state_dim_);
          force_pos_.resize(state_dim_);
          force_vel_.resize(state_dim_);
          final_state_.resize(state_dim_);
          initial_state_.resize(state_dim_);
          t_versor_.resize(state_dim_);
          J_.resize(state_dim_,1);
          J_transp_.resize(1,state_dim_);
          BxJ_.resize(state_dim_,1);
          JtxBxJ_.resize(1,1); // NOTE It is used to store the multiplication J * J_transp

          fade_sys_.SetRef(1.0);

          // Default quaternions
          //q_start_.reset(new Eigen::Quaternion(1.0,0.0,0.0,0.0));
          //q_end_.reset(new Eigen::Quaternion(1.0,0.0,0.0,0.0));
          //quaternion_ << 1.0,0.0,0.0,0.0;
	  }
	
      //VirtualMechanismInterface(const VirtualMechanismInterface& to_copy); // copy constructor
      virtual VirtualMechanismInterface* Clone() = 0;

	  virtual ~VirtualMechanismInterface()
      {
      }
	  
      inline bool ReadConfig()
      {
          YAML::Node main_node = tool_box::CreateYamlNodeFromPkgName(ROS_PKG_NAME);
          if (const YAML::Node& curr_node = main_node["virtual_mechanism_interface"])
          {
              std::vector<double> K,B;
              curr_node["K"] >> K;
              curr_node["B"] >> B;
              curr_node["n_points_discretization"] >> n_points_discretization_;

              assert(n_points_discretization_ > 1);

              state_dim_ = K.size();
              assert(state_dim_ == 2 || state_dim_ == 3);
              //assert(K.size() == static_cast<unsigned int>(state_dim_));
              assert(B.size() == K.size());
              for(unsigned int i=0; i<K.size(); i++)
              {
                assert(K[i] > 0.0);
                assert(B[i] > 0.0);
              }

              // Create a diagonal gain matrix
              if(K.size() == 2)
              {
                K_ = Eigen::DiagonalMatrix<double,2>(K[0],K[1]);
                B_ = Eigen::DiagonalMatrix<double,2>(B[0],B[1]);
              }
              else if(K.size() == 3)
              {
                K_ = Eigen::DiagonalMatrix<double,3>(K[0],K[1],K[2]);
                B_ = Eigen::DiagonalMatrix<double,3>(B[0],B[1],B[2]);
              }

              if (const YAML::Node& active_guide_node = curr_node["active_guide"])
              {
                  double fade_sys_gain;
                  active_guide_node["Kf"] >> Kf_;
                  active_guide_node["Bf"] >> Bf_;
                  active_guide_node["fade_sys_gain"] >> fade_sys_gain;
                  assert(Kf_ >= 0.0);
                  assert(Bf_ >= 0.0);
                  fade_sys_.SetGain(fade_sys_gain);
                  check_activation_ = true;
              }
              else
              {
                  Kf_ = 0.0;
                  Bf_ = 0.0;
                  check_activation_ = false;
              }
              return true;
          }
          else
              return false;
      }

      inline void CheckForActivation();

      virtual void Update(Eigen::VectorXd& force, const double dt)
	  {
        assert(dt > 0.0);

        dt_ = dt;

	    // Save the previous phase
	    phase_prev_ = phase_;

        // Save the previous phase_dot
        phase_dot_prev_ = phase_dot_;

        // Check for guide activation
        if(check_activation_)
            CheckActivation();
  
	    // Update the Jacobian and its transpose
	    UpdateJacobian();
	    
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

        // Compute the jacobian versor (used to avoid the lock in the manager)
        ComputeJacobianVersor();
	  }

      void UpdateDiscrete(const Eigen::VectorXd& pos)
      {

        phase_dot_ = 0.0;
        phase_ddot_ = 0.0;

        // Update the Jacobian and its transpose
        UpdateJacobian();

        // Compute the phase based on the min distance
        FindMinDist(pos);

        // Compute the new state
        UpdateState();

        // Compute the new state dot
        UpdateStateDot();
      }
	  
      void FindMinDist(const Eigen::VectorXd& pos)
      {
          assert(state_recorded_.rows() > 0);
          assert(pos.size() ==  state_recorded_.cols());
          assert(tmp_dists_.size() == state_recorded_.rows());
          assert(phase_recorded_.rows() ==  state_recorded_.rows());
          assert(phase_recorded_.cols() ==  1);

          int min_idx = 0;
          double curr_d;
          double min = std::numeric_limits<double>::infinity();
          for(unsigned int i_row = 0; i_row < state_recorded_.rows(); i_row++)
          {
              curr_d = (state_recorded_.row(i_row).transpose() - pos).norm();
              if(curr_d < min)
              {
                  min = curr_d;
                  min_idx = i_row;
              }


          }

          /*std::cout << "***" << std::endl;

          std::cout << pos << std::endl;

          std::cout << "***" << std::endl;
          std::cout << state_recorded_.row(0).transpose() - pos << std::endl;*/

          phase_ = phase_recorded_(min_idx,0);
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

          if(scale_ > 0.01) // Compute the movement of the mechanism
          {
              //K_ = adaptive_gain_ptr_->ComputeGain((state_ - pos).norm());
              displacement_.noalias() = state_ - pos;
              force_pos_.noalias() = K_ * displacement_;
              force_vel_.noalias() = B_ * vel;
              force_ = force_pos_ - force_vel_;
              //force_ = force_;
              //force_ = scale * (K_ * (state_ - pos) - B_ * (vel));
              Update(force_,dt);
          }
          else // Find the min distance point
          {
              UpdateDiscrete(pos);
          }

          // Publish stuff
#ifdef USE_ROS_RT_PUBLISHER
          rt_publishers_.PublishAll();
#endif
	  }
	  
      // Here to no break the polymorphism
      virtual double ComputeResponsability(const Eigen::MatrixXd& pos){PRINT_ERROR("ComputeResponsability has not been defined.");}
      virtual double GetResponsability(){PRINT_ERROR("GetResponsability has not been defined.");}

      virtual bool CreateModelFromData(const Eigen::MatrixXd& data)=0;
      virtual bool CreateModelFromFile(const std::string file_path)=0;
      virtual bool SaveModelToFile(const std::string file_path)=0;

      virtual double getDistance(const Eigen::VectorXd& pos)=0;
      virtual double getScale(const Eigen::VectorXd& pos, const double convergence_factor = 1.0)=0;

      inline double getTorque() const {return torque_(0,0);}
      inline double getFade() const {return fade_;}
      inline double getPhaseDotDot() const {return phase_ddot_;}
      inline double getPhaseDot() const {return phase_dot_;}
      inline double getPhase() const {return phase_;}
      inline double getPhaseRef() const {return phase_ref_;}
      inline double getPhaseDotRef() const {return phase_dot_ref_;}
      inline double getPhaseDotDotRef() const {return phase_ddot_ref_;}

      inline double getKf() const {return Kf_;}
      inline double getBf() const {return Bf_;}

      inline void getJacobianVersor(Eigen::VectorXd& t_versor) const {assert(t_versor.size() == state_dim_); t_versor = t_versor_;}
      inline void getInitialPos(Eigen::VectorXd& state) const {assert(state.size() == state_dim_); state = initial_state_;}
      inline void getFinalPos(Eigen::VectorXd& state) const {assert(state.size() == state_dim_); state = final_state_;}
      inline void getState(Eigen::VectorXd& state) const {assert(state.size() == state_dim_); state = state_;}
	  inline void getStateDot(Eigen::VectorXd& state_dot) const {assert(state_dot.size() == state_dim_); state_dot = state_dot_;}
      inline void getJacobian(Eigen::MatrixXd& jacobian) const {jacobian = J_;}
      inline void getK(Eigen::MatrixXd& K) const {K = K_;}
      inline void getB(Eigen::MatrixXd& B) const {B = B_;}
      inline void getQuaternion(Eigen::VectorXd& q) const
      {
              assert(q.size() == 4);
              q(0) = quaternion_->w();
              q(1) = quaternion_->x();
              q(2) = quaternion_->y();
              q(3) = quaternion_->z();
      }

      inline Eigen::VectorXd& getJacobianVersor() {return t_versor_;}
      inline Eigen::VectorXd& getInitialPos() {return initial_state_;}
      inline Eigen::VectorXd& getFinalPos() {return final_state_;}
      inline Eigen::VectorXd& getState() {return state_;}
      inline Eigen::VectorXd& getStateDot() {return state_dot_;}
      inline Eigen::MatrixXd& getJacobian() {return J_;}
      inline Eigen::MatrixXd& getK() {return K_;}
      inline Eigen::MatrixXd& getB() {return B_;}

      //inline void setExecutionTime(const double time) {assert(time > 0.0); exec_time_ = time;}
      inline void setCollisionDetected(const bool collision) {collision_detected_ = collision;}

      inline void Init()
      {
          // Initialize the attributes
          UpdateJacobian();
          UpdateState();
          UpdateStateDot();
          ComputeInitialState();
          ComputeFinalState();
          ComputeJacobianVersor();
          CreateRecordedRefs();
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

      inline void InitRtPublishers(const std::string node_name)
      {

#ifdef USE_ROS_RT_PUBLISHER

        // Reset the ros node (usefull if we change node's name)
        if(ros_node_.InitDone())
            ros_node_.Reset();

        // Initialize the ros node and the rt-publishers
        try
        {
              ros_node_.Init(node_name);
              rt_publishers_.AddPublisher(ros_node_.GetNode(),"phase",&phase_);
              rt_publishers_.AddPublisher(ros_node_.GetNode(),"phase_dot",&phase_dot_);
              rt_publishers_.AddPublisher(ros_node_.GetNode(),"phase_dot",&phase_ddot_);
              rt_publishers_.AddPublisher(ros_node_.GetNode(),"scale",&scale_);
              rt_publishers_.AddPublisher(ros_node_.GetNode(),"phase_dot_ref",&phase_dot_ref_);
        }
        catch(const std::runtime_error& e)
        {
              ROS_ERROR("Failed to create the real time publishers: %s",e.what());
        }
#else
        PRINT_WARNING("Impossible to start the real time publishers.");
#endif
      }

   protected:

	  virtual void UpdateJacobian()=0;
	  virtual void UpdateState()=0;
	  virtual void UpdatePhase(const Eigen::VectorXd& force, const double dt)=0;
	  virtual void ComputeInitialState()=0;
	  virtual void ComputeFinalState()=0;
      virtual void CreateRecordedRefs()=0;

      inline void ComputeJacobianVersor()
      {
          // Jacobian versor
          t_versor_ = J_/J_.norm();
      }

      inline void CheckActivation()
      {
          autom_.Step(phase_dot_,phase_dot_ref_,collision_detected_);
          if(autom_.GetState())
              active_ = true;
          else
              active_ = false;
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

      virtual inline void UpdateStateDot()
	  {
          state_dot_.noalias() = J_ * phase_dot_;
	  }
	  
	  inline void UpdateQuaternion()
      {
          *quaternion_ = q_start_->slerp(phase_,*q_end_);
      }
	  
      /// States
	  double phase_;
	  double phase_prev_;
	  double phase_dot_;
      double phase_dot_ref_;
      double phase_ddot_ref_;
      double phase_ref_;
      double phase_dot_prev_;
      double phase_ddot_;
      double scale_;
      int state_dim_;
      Eigen::VectorXd displacement_;
      Eigen::VectorXd state_;
      Eigen::VectorXd state_dot_;
	  Eigen::VectorXd torque_;
      Eigen::VectorXd force_;
      Eigen::VectorXd force_out_;
      Eigen::VectorXd force_pos_;
      Eigen::VectorXd force_vel_;
	  Eigen::VectorXd initial_state_;
	  Eigen::VectorXd final_state_;
      Eigen::VectorXd t_versor_;
      Eigen::ArrayXd tmp_dists_;
      Eigen::MatrixXd BxJ_;
      Eigen::MatrixXd JtxBxJ_;
	  Eigen::MatrixXd J_;
	  Eigen::MatrixXd J_transp_;

<<<<<<< HEAD
      /// Gains
=======
      // Discretization
      int n_points_discretization_;
      Eigen::MatrixXd state_recorded_;
      Eigen::MatrixXd phase_recorded_;

	  // Gains
>>>>>>> a3a3acf25123d3eac6c12bc580bd79b5a6408cf0
      Eigen::MatrixXd B_;
      Eigen::MatrixXd K_;

      /// Fade system
      tool_box::DynSystemFirstOrder fade_sys_;
	  
      /// Auto completion
	  double Kf_;
      double Bf_;
	  double fade_;
	  bool active_;
      bool check_activation_;
      bool collision_detected_;
      double dt_;
      VirtualMechanismAutom autom_;

      /// Orientation
      bool update_quaternion_;
      boost::shared_ptr<quaternion_t > q_start_;
      boost::shared_ptr<quaternion_t > q_end_;
      boost::shared_ptr<quaternion_t > quaternion_;

      /// Ros Stuff
#ifdef USE_ROS_RT_PUBLISHER
      tool_box::RosNode ros_node_;
      tool_box::RealTimePublishers<tool_box::RealTimePublisherScalar> rt_publishers_;
#endif

};
  
class VirtualMechanismInterfaceFirstOrder : public VirtualMechanismInterface
{
	public:
      VirtualMechanismInterfaceFirstOrder():
      VirtualMechanismInterface()
	  {

        if(!ReadConfig())
        {
          PRINT_ERROR("VirtualMechanismInterfaceFirstOrder: Can not read config file");
        }

	    det_ = 1.0;
        num_ = -1.0;
	  }

      inline bool ReadConfig()
      {
          YAML::Node main_node = tool_box::CreateYamlNodeFromPkgName(ROS_PKG_NAME);
          if (const YAML::Node& curr_node = main_node["first_order"])
          {
              //main_node["Bd_max"] >> Bd_max_;
              //main_node["epsilon"] >> epsilon_;
              //assert(epsilon > 0.1);
              curr_node["Bd"] >> Bd_;
              return true;
          }
          else
              return false;
      }

	protected:
	    
	  virtual void UpdateJacobian()=0;
	  virtual void UpdateState()=0;
	  virtual void ComputeInitialState()=0;
	  virtual void ComputeFinalState()=0;
	  
	  virtual void UpdatePhase(const Eigen::VectorXd& force, const double dt)
	  {
          BxJ_.noalias() = B_ * J_;
          JtxBxJ_.noalias() = J_transp_ * BxJ_;

	      // Adapt Bf
          /*Bd_ = std::exp(-4/epsilon_*JxJt_(0,0)) * Bd_max_; // NOTE: Since JxJt_ has dim 1x1 the determinant is the only value in it
	      //Bf_ = std::exp(-4/epsilon_*JxJt_.determinant()) * Bf_max_; // NOTE JxJt_.determinant() is always positive! so it's ok
          det_ = B_ * JxJt_(0,0) + Bd_ * Bd_;*/

          det_ = JtxBxJ_(0,0) + Bd_;

	      torque_.noalias() = J_transp_ * force;
	      
          if(active_)
              fade_sys_.IntegrateForward(dt);
             //fade_ = fade_gain_ * (1 - fade_) * dt + fade_;
          else
             fade_sys_.IntegrateBackward(dt);
             //fade_ = fade_gain_ * (-fade_) * dt + fade_;

          fade_ = fade_sys_.GetState();

          // Always keep the external torque
          //phase_dot_ = num_/det_ * torque_(0,0) + fade_ * (Kf_ * (phase_ref_ - phase_) + Bf_ * phase_dot_ref_);

          // Switch between open and closed loop with the external torque
          phase_dot_ = num_/det_ * torque_(0,0);
          phase_dot_ = fade_ *  phase_dot_ref_ + (1-fade_) * phase_dot_;

	      // Compute the new phase
          phase_ = phase_dot_ * dt + phase_prev_;

           // Compute phase_ddot
          phase_ddot_ = (phase_dot_ - phase_dot_prev_)/dt;
	  }

	  double det_;
      double num_;
	  double Bd_; // Damp term
      //double Bd_max_; // Max damp term
      //double epsilon_;
};

class VirtualMechanismInterfaceSecondOrder : public VirtualMechanismInterface
{
	public:
      VirtualMechanismInterfaceSecondOrder():
      VirtualMechanismInterface()
      {
          if(!ReadConfig())
          {
            PRINT_ERROR("VirtualMechanismInterfaceSecondOrder: Can not read config file");
          }

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
	  }

      inline bool ReadConfig()
      {
          YAML::Node main_node = tool_box::CreateYamlNodeFromPkgName(ROS_PKG_NAME);
          if (const YAML::Node& curr_node = main_node["second_order"])
          {
              curr_node["inertia"] >> inertia_;
              assert(inertia_ > 0.0);
              return true;
          }
          else
              return false;
      }
	
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
         phase_state_dot_(1) = (1/inertia_)*(- JtxBxJ_(0,0) * phase_state(1) - input1 + input2); // Old version with damping
         phase_state_dot_(0) = phase_state(1);

         //phase_state_dot_(1) = (1/inertia_)*(- JtxBxJ_(0,0) * phase_state(1) - input1); // Old version with damping
         //phase_state_dot_(1) = (1/inertia_)*(- (B_ * JxJt_(0,0)  + F ) * phase_state(1) - input); // Version with friction
         //phase_state_dot_(1) = (1/inertia_)*(-input - 0.1 * phase_state(1)); // double integrator with friction
         //phase_state_dot_(1) = (1/inertia_)*(-(B_ * JxJt_(0,0) + Bf_) * phase_state(1) - input + fade_ *  (Bf_ * phase_dot_ref_ + Kf_ * (phase_ref_ - phase_state(0)))); // Joly
         //phase_state_dot_(1) = (1/inertia_)*(-0.00001 * phase_state(1) - input + fade_ *  (Bf_ * phase_dot_ref_ + Kf_ * (phase_ref_ - phase_state(0))));
         //phase_state_dot_(1) = (1/inertia_)*(- input + fade_ *  (Bf_ * (phase_dot_ref_ -  phase_state(1)) + Kf_ * (phase_ref_ - phase_state(0)))); // Working no friction, pure double integrator
         //phase_state_dot_(1) = (1/inertia_)*(- input - B_ * JxJt_(0,0) * phase_state(1) + fade_ *  (Bf_ * (phase_dot_ref_ -  phase_state(1)) + Kf_ * (phase_ref_ - phase_state(0))));
         //phase_state_dot_(1) = (1/inertia_)*( - input1 - 0.1 * phase_state(1) + input2 ); // FIXME 0.1 is just a little friction to avoid instability
         //phase_state_dot_(1) = (1/inertia_)*( - input1 - (1.0 - scale_) * 1.0 * phase_state(1) + input2 ); // dynamic brakes!
         //phase_state_dot_(0) = fade_ *  phase_dot_ref_  + (1-fade_) * phase_state(1);
	  }
	  
	  virtual void UpdatePhase(const Eigen::VectorXd& force, const double dt)
	  {
          BxJ_.noalias() = B_ * J_;
          JtxBxJ_.noalias() = J_transp_ * BxJ_;

	      torque_.noalias() = J_transp_ * force;

          phase_state_(0) = phase_;
          phase_state_(1) = phase_dot_;
	        
          if(active_)
              fade_sys_.IntegrateForward(dt);
             //fade_ = fade_gain_ * (1 - fade_) * dt + fade_;
          else
             fade_sys_.IntegrateBackward(dt);
             //fade_ = fade_gain_ * (-fade_) * dt + fade_;

          fade_ = fade_sys_.GetState();

          control_ = fade_ * (Bf_ * (phase_dot_ref_ - phase_dot_) + Kf_ * (phase_ref_ - phase_));
	      
          IntegrateStepRungeKutta(dt,torque_(0),control_,phase_state_,phase_state_integrated_);

          DynSystem(dt,torque_(0),control_,phase_state_); // to compute the dots

          phase_ = phase_state_integrated_(0);
	      phase_dot_ = phase_state_integrated_(1);
          phase_ddot_ = phase_state_dot_(1);
	  }
	  
	  Eigen::VectorXd phase_state_;
	  Eigen::VectorXd phase_state_dot_;
	  Eigen::VectorXd phase_state_integrated_;
	  Eigen::VectorXd k1_, k2_, k3_, k4_;
      double inertia_;
      double control_;
};

}

#endif
