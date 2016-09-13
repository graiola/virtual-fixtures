/**
 * @file   virtual_mechanism_gmr.h
 * @brief  Virtual mechanisms implemented using GMM/GMR.
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

#ifndef VIRTUAL_MECHANISM_GMR_H
#define VIRTUAL_MECHANISM_GMR_H

////////// VirtualMechanismInterface
#include <virtual_mechanism/virtual_mechanism_interface.h>

////////// Function Approximator
//#include <functionapproximators/FunctionApproximatorGMR.hpp>
//#include <functionapproximators/ModelParametersGMR.hpp>
//#include <functionapproximators/MetaParametersGMR.hpp>
#include <vf_gmr/FunctionApproximatorGMR.hpp>
#include <vf_gmr/ModelParametersGMR.hpp>
#include <vf_gmr/MetaParametersGMR.hpp>

////////// BOOST
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

////////// Toolbox
#include "toolbox/spline/spline.h"
#include "toolbox/dtw/dtw.h"

namespace virtual_mechanism
{
  
  typedef DmpBbo::FunctionApproximatorGMR fa_t;

template <class VM_t>  
class VirtualMechanismGmr: public VM_t
{
	public:

      VirtualMechanismGmr();
      VirtualMechanismGmr(const std::string file_path);
      VirtualMechanismGmr(const Eigen::MatrixXd& data);
      VirtualMechanismGmr(const fa_t* const fa);
      ~VirtualMechanismGmr();

      virtual VirtualMechanismInterface* Clone();

      virtual double getDistance(const Eigen::VectorXd& pos);
      virtual double getScale(const Eigen::VectorXd& pos, const double convergence_factor = 1.0);
      virtual bool CreateModelFromData(const Eigen::MatrixXd& data);
      virtual bool CreateModelFromFile(const std::string file_path);
      virtual bool SaveModelToFile(const std::string file_path);

      void ComputeStateGivenPhase(const double abscisse_in, Eigen::VectorXd& state_out);
      void AlignAndUpateGuide(const Eigen::MatrixXd& data);
      double ComputeResponsability(const Eigen::MatrixXd& pos);
      double GetResponsability();
	  
	protected:
	  
      bool ReadConfig();
      void TrainModel(const Eigen::MatrixXd& data);
	  virtual void UpdateJacobian();
	  virtual void UpdateState();
	  virtual void ComputeInitialState();
      virtual void ComputeFinalState();
      virtual void CreateRecordedRefs();

      void UpdateInvCov();
      double ComputeProbability(const Eigen::VectorXd& pos);

      fa_t* fa_; // Function Approximator

	  Eigen::MatrixXd fa_input_;
	  Eigen::MatrixXd fa_output_;
	  Eigen::MatrixXd fa_output_dot_;
	  Eigen::MatrixXd variance_;
	  Eigen::MatrixXd covariance_;
      Eigen::MatrixXd covariance_inv_;
	  Eigen::VectorXd err_;

      int n_gaussians_;
};

template <typename VM_t>
class VirtualMechanismGmrNormalized: public VirtualMechanismGmr<VM_t>
{
    public:

      VirtualMechanismGmrNormalized();
      VirtualMechanismGmrNormalized(const std::string file_path);
      VirtualMechanismGmrNormalized(const Eigen::MatrixXd& data);
      VirtualMechanismGmrNormalized(const fa_t* const fa);

      virtual VirtualMechanismInterface* Clone();

      void ComputeStateGivenPhase(const double phase_in, Eigen::VectorXd& state_out, Eigen::VectorXd& state_out_dot, double& phase_out, double& phase_out_dot);
      void AlignAndUpateGuide(const Eigen::MatrixXd& data);

      virtual bool CreateModelFromData(const Eigen::MatrixXd& data);
      virtual bool CreateModelFromFile(const std::string file_path);

    protected:

      bool ReadConfig();
      void TrainModel(const Eigen::MatrixXd& data);
      virtual void UpdateJacobian();
      virtual void UpdateState();
      virtual void UpdateStateDot();
      void Normalize();

      tk::spline spline_phase_;
      tk::spline spline_phase_inv_;
      std::vector<tk::spline > splines_xyz_;
      bool use_spline_xyz_;
      int n_points_splines_;
      double exec_time_;

      double z_;
      double z_dot_;
      double z_dot_ref_;

      Eigen::MatrixXd Jz_;

      long long loopCnt;
};

}

#endif
