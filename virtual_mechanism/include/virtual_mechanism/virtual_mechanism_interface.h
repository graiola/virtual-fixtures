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
#include <vf_toolbox/vf_toolbox.h>

#define LINE_CLAMP(x,y,x1,x2,y1,y2) do { y = (y2-y1)/(x2-x1) * (x-x1) + y1; } while (0)

namespace virtual_mechanism
{

class VirtualMechanismInterface
{
public:
  VirtualMechanismInterface():
    scale_(1.0),
    dt_(0.001)
  {

  }

  inline void Resize(int state_dim,int phase_dim)
  {
    // Initialize/resize the attributes
    phase_.resize(phase_dim);
    phase_prev_.resize(phase_dim);
    phase_dot_.resize(phase_dim);
    phase_dot_prev_.resize(phase_dim);
    phase_ddot_.resize(phase_dim);
    torque_.resize(phase_dim);

    /*phase_.fill(0.0);
    phase_prev_.fill(0.0);
    phase_dot_.fill(0.0);
    phase_dot_prev_.fill(0.0);
    phase_ddot_.fill(0.0);*/

    state_.resize(state_dim_);
    state_dot_.resize(state_dim_);
    displacement_.resize(state_dim_);
    force_.resize(state_dim_);
    force_pos_.resize(state_dim_);
    force_vel_.resize(state_dim_);
    final_state_.resize(state_dim_);
    initial_state_.resize(state_dim_);
    t_versor_.resize(state_dim_);
    J_.resize(state_dim_,phase_dim);
    J_transp_.resize(phase_dim,state_dim_);
    BxJ_.resize(state_dim_,phase_dim);
    JtxBxJ_.resize(phase_dim,phase_dim); // NOTE It is used to store the multiplication J * J_transp

    K_.resize(state_dim_,state_dim_);
    B_.resize(state_dim_,state_dim_);

    // TODO: Create a new object with the gains K B and M (inertia)
    // Export this object!
    // Create diagonal gain matrices
    double k = 1000.0;
    double b = 100.0;
    if(state_dim_ == 2)
    {
      K_ = Eigen::DiagonalMatrix<double,2>(k,k);
      B_ = Eigen::DiagonalMatrix<double,2>(b,b);
    }
    else if(state_dim_ == 3)
    {
      K_ = Eigen::DiagonalMatrix<double,3>(k,k,k);
      B_ = Eigen::DiagonalMatrix<double,3>(b,b,b);
    }

  }

  //virtual VirtualMechanismInterface* Clone() = 0;

  virtual ~VirtualMechanismInterface()
  {
  }

  /* inline bool GetParameters()
  {
    std::vector<double> K,B;

    if (nh_.hasParam("K"))
      nh_.getParam("K", K);
    else
    {
      ROS_INFO_STREAM("VirtualMechanismInterface: K not found on parameter server ("<<nh_.getNamespace()<<"/K), using default values");
      K.resize(state_dim_);
      std::fill(K.begin(), K.end(), 100.0);
    }
    if (nh_.hasParam("B"))
      nh_.getParam("B", K);
    else
    {
      ROS_INFO_STREAM("VirtualMechanismInterface: B not found on parameter server ("<<nh_.getNamespace()<<"/B), using default values");
      B.resize(state_dim_);
      std::fill(B.begin(), B.end(), 10.0);
    }
    if (nh_.hasParam("n_points_discretization"))
      nh_.getParam("n_points_discretization", n_points_discretization_);
    else
    {
      ROS_INFO_STREAM("VirtualMechanismInterface: n_points_discretization not found on parameter server ("<<nh_.getNamespace()<<"/n_points_discretization), using default value");
      // TODO compute the number of points smartly
      n_points_discretization_ = 1000;
    }

    assert(n_points_discretization_ > 1);

    int state_dim = K.size();
    assert(state_dim == 2 || state_dim == 3);
    assert(B.size() == state_dim);
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

    return true;
  }*/

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

    // Update the phase
    UpdatePhase(force,dt);

    // Saturate the phase if exceeds 1 or 0
    ApplySaturation();

    // Compute the new state
    UpdateState();

    // Compute the new state dot
    UpdateStateDot();

    // Compute the jacobian versor (used to avoid the lock in the manager)
    ComputeJacobianVersor();
  }

  /*void UpdateDiscrete(const Eigen::VectorXd& pos)
  {
    phase_dot_.fill(0.0);
    phase_ddot_.fill(0.0);

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
    phase_ = phase_recorded_(min_idx,0);
  }*/

  virtual void Stop()
  {
    phase_dot_.fill(0.0);
    phase_ddot_.fill(0.0);
  }

  inline void Update(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel , const double dt, const double scale = 1.0)
  {
    assert(pos.size() == state_dim_);
    assert(vel.size() == state_dim_);

    scale_ = scale;

    if(scale_ > 0.01) // Compute the movement of the mechanism
    {
      displacement_.noalias() = pos - state_;
      force_pos_.noalias() = K_ * displacement_;
      force_vel_.noalias() = B_ * vel;
      force_ = force_pos_ + force_vel_;
      Update(force_,dt);
    }
    else // Find the min distance point
    {
      // TODO
      //UpdateDiscrete(pos);
    }

  }


  virtual double getDistance(const Eigen::VectorXd& pos)=0;
  virtual double getScale(const Eigen::VectorXd& pos, const double convergence_factor = 1.0)=0;

  inline void getTorque(Eigen::VectorXd& torque) const {assert(torque.size() == phase_dim_); torque = torque_;}
  inline void getPhaseDotDot(Eigen::VectorXd& phase_ddot) const {assert(phase_ddot.size() == phase_dim_); phase_ddot = phase_ddot_;}
  inline void getPhaseDot(Eigen::VectorXd& phase_dot) const {assert(phase_dot.size() == phase_dim_); phase_dot = phase_dot_;}
  inline void getPhase(Eigen::VectorXd& phase) const {assert(phase.size() == phase_dim_); phase = phase_;}
  inline Eigen::VectorXd& getTorque() {return torque_;}
  inline Eigen::VectorXd& getPhaseDotDot() {return phase_ddot_;}
  inline Eigen::VectorXd& getPhaseDot() {return phase_dot_;}
  inline Eigen::VectorXd& getPhase() {return phase_;}

  inline void getJacobianVersor(Eigen::VectorXd& t_versor) const {assert(t_versor.size() == state_dim_); t_versor = t_versor_;}
  inline void getInitialPos(Eigen::VectorXd& state) const {assert(state.size() == state_dim_); state = initial_state_;}
  inline void getFinalPos(Eigen::VectorXd& state) const {assert(state.size() == state_dim_); state = final_state_;}
  inline void getState(Eigen::VectorXd& state) const {assert(state.size() == state_dim_); state = state_;}
  inline void getStateDot(Eigen::VectorXd& state_dot) const {assert(state_dot.size() == state_dim_); state_dot = state_dot_;}
  inline void getJacobian(Eigen::MatrixXd& jacobian) const {jacobian = J_;}
  inline void getK(Eigen::MatrixXd& K) const {K = K_;}
  inline void getB(Eigen::MatrixXd& B) const {B = B_;}
  inline Eigen::VectorXd& getJacobianVersor() {return t_versor_;}
  inline Eigen::VectorXd& getInitialPos() {return initial_state_;}
  inline Eigen::VectorXd& getFinalPos() {return final_state_;}
  inline Eigen::VectorXd& getState() {return state_;}
  inline Eigen::VectorXd& getStateDot() {return state_dot_;}
  inline Eigen::MatrixXd& getJacobian() {return J_;}
  inline Eigen::MatrixXd& getK() {return K_;}
  inline Eigen::MatrixXd& getB() {return B_;}

  inline void Init()
  {
    // Initialize the attributes
    UpdateJacobian();
    UpdateState();
    UpdateStateDot();
    ComputeJacobianVersor();
    //CreateRecordedRefs();
  }

protected:

  virtual void UpdateJacobian()=0;
  virtual void UpdateState()=0;
  virtual void UpdatePhase(const Eigen::VectorXd& force, const double dt)=0;
  //virtual void CreateRecordedRefs()=0;

  inline void ComputeJacobianVersor()
  {
    // Jacobian versor
    t_versor_ = J_/J_.norm();
  }

  virtual void ApplySaturation()
  {
    for(int i = 0; i < phase_.size(); i++)
    {
      // Saturations
      if (phase_(i) > 1.0)
      {
        //LINE_CLAMP(phase_(i),clamp_,0.9,1,1,0);
        phase_(i) = 1.0;
        phase_dot_(i) = 0.0;
        phase_ddot_(i) = 0.0;
      }
      else if (phase_(i) < 0.0)
      {
        //LINE_CLAMP(phase_(i),clamp_,0,0.1,0,1);
        phase_(i) = 0.0;
        phase_dot_(i) = 0.0;
        phase_ddot_(i) = 0.0;
      }
    }
  }

  virtual inline void UpdateStateDot()
  {
    state_dot_.noalias() = J_ * phase_dot_;
  }

  // States
  Eigen::VectorXd phase_;
  Eigen::VectorXd phase_prev_;
  Eigen::VectorXd phase_dot_;
  Eigen::VectorXd phase_dot_prev_;
  Eigen::VectorXd phase_ddot_;
  double scale_;
  double dt_;
  int state_dim_;
  int phase_dim_;
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

  // Discretization
  int n_points_discretization_;
  Eigen::MatrixXd state_recorded_;
  Eigen::MatrixXd phase_recorded_;

  // Gains
  Eigen::MatrixXd B_;
  Eigen::MatrixXd K_;

};

class VirtualMechanismInterfaceFirstOrder : public VirtualMechanismInterface
{
public:
  VirtualMechanismInterfaceFirstOrder():
    VirtualMechanismInterface()
  {
  }

  virtual void Resize(int state_dim, int phase_dim)
  {
    assert(state_dim == 2 || state_dim == 3);
    state_dim_ = state_dim;
    assert(phase_dim >= 1 || phase_dim <= 3);
    phase_dim_ = phase_dim;
    JtxBxJ_inv_.resize(phase_dim,phase_dim);
    Bd_.resize(state_dim,state_dim);
    VirtualMechanismInterface::Resize(state_dim,phase_dim);
  }

protected:

  virtual void UpdateJacobian()=0;
  virtual void UpdateState()=0;

  virtual void UpdatePhase(const Eigen::VectorXd& force, const double dt)
  {
    BxJ_.noalias() = B_ * J_;
    JtxBxJ_.noalias() = J_transp_ * BxJ_;

    //JtxBxJ_ = JtxBxJ_ + Bd_;

    //JtxBxJ_inv_ = JtxBxJ_.inverse();

    torque_.noalias() = J_transp_ * force;

    // Compute phase dot given the input torque
    phase_dot_ = JtxBxJ_.inverse() * torque_;

    // Compute the new phase
    phase_ = phase_dot_ * dt + phase_prev_;

    // Compute phase_ddot
    phase_ddot_ = (phase_dot_ - phase_dot_prev_)/dt;
  }

  Eigen::MatrixXd JtxBxJ_inv_;
  Eigen::MatrixXd Bd_; // Damp term
};

class VirtualMechanismInterfaceSecondOrder : public VirtualMechanismInterface
{
public:
  VirtualMechanismInterfaceSecondOrder():
    VirtualMechanismInterface()
  {
  }

  virtual void Resize(int state_dim, int phase_dim)
  {
    assert(state_dim == 2 || state_dim == 3);
    state_dim_ = state_dim;
    assert(phase_dim >= 1 || phase_dim <= 3);
    phase_dim_ = phase_dim;

    // Resize the attributes for the integration
    phase_state_.resize(2*phase_dim); //phase_ and phase_dot
    phase_state_dot_.resize(2*phase_dim); //phase_dot and phase_ddot
    phase_state_integrated_.resize(2*phase_dim);

    phase_state_.fill(0.0);
    phase_state_dot_.fill(0.0);
    phase_state_integrated_.fill(0.0);

    k1_.resize(2*phase_dim);
    k2_.resize(2*phase_dim);
    k3_.resize(2*phase_dim);
    k4_.resize(2*phase_dim);

    k1_.fill(0.0);
    k2_.fill(0.0);
    k3_.fill(0.0);
    k4_.fill(0.0);

    M_.resize(phase_dim,phase_dim); // TODO

    VirtualMechanismInterface::Resize(state_dim,phase_dim);
  }

protected:

  virtual void UpdateJacobian()=0;
  virtual void UpdateState()=0;

  void IntegrateStepRungeKutta(const double& dt, const Eigen::VectorXd& input, const Eigen::VectorXd& phase_state, Eigen::VectorXd& phase_state_integrated)
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

  inline void DynSystem(const double& dt, const Eigen::VectorXd& torque, const Eigen::VectorXd& phase_state)
  {
    phase_state_dot_.segment(phase_dim_,phase_dim_) = M_.inverse()*(- JtxBxJ_ * phase_state.segment(phase_dim_,phase_dim_) - torque);
    phase_state_dot_.segment(0,phase_dim_) = phase_state.segment(phase_dim_,phase_dim_);
  }

  virtual void UpdatePhase(const Eigen::VectorXd& force, const double dt)
  {
    BxJ_.noalias() = B_ * J_;
    JtxBxJ_.noalias() = J_transp_ * BxJ_;

    torque_.noalias() = J_transp_ * force;

    phase_state_.segment(0,phase_dim_) = phase_;
    phase_state_.segment(phase_dim_,phase_dim_) = phase_dot_;

    IntegrateStepRungeKutta(dt,torque_,phase_state_,phase_state_integrated_);

    DynSystem(dt,torque_,phase_state_); // to compute the dots

    phase_ = phase_state_integrated_.segment(0,phase_dim_);
    phase_dot_ = phase_state_integrated_.segment(phase_dim_,phase_dim_);
    phase_ddot_ = phase_state_dot_.segment(phase_dim_,phase_dim_);
  }

  Eigen::VectorXd phase_state_;
  Eigen::VectorXd phase_state_dot_;
  Eigen::VectorXd phase_state_integrated_;
  Eigen::VectorXd k1_, k2_, k3_, k4_;
  Eigen::MatrixXd M_; //virtual inertia matrix
};

}

#endif
