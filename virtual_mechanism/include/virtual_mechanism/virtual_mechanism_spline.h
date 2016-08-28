/**
 * @file   virtual_mechanism_spline.h
 * @brief  Virtual mechanisms implemented using splines.
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

#ifndef VIRTUAL_MECHANISM_SPLINE_H
#define VIRTUAL_MECHANISM_SPLINE_H

////////// VirtualMechanismInterface
#include <virtual_mechanism/virtual_mechanism_interface.h>

////////// Toolbox
#include "toolbox/spline/spline.h"

namespace virtual_mechanism
{ 

template <class VM_t>  
class VirtualMechanismSpline: public VM_t
{
	public:

      VirtualMechanismSpline();
      VirtualMechanismSpline(const std::string file_path);
      VirtualMechanismSpline(const Eigen::MatrixXd& data);
	  
      virtual double getDistance(const Eigen::VectorXd& pos);
      virtual double getScale(const Eigen::VectorXd& pos, const double convergence_factor = 1.0);
      virtual bool SaveModelToFile(const std::string file_path);
      void ComputeStateGivenPhase(const double phase_in, Eigen::VectorXd& state_out);
	  
	protected:

      bool LoadModelFromFile(const std::string file_path);
	  
      virtual void UpdateJacobian();
      virtual void UpdateState();
      virtual void UpdateStateDot();
      virtual void ComputeInitialState();
      virtual void ComputeFinalState();

      std::vector<tk::spline > splines_xyz_;
      tk::spline spline_phase_;
      tk::spline spline_phase_inv_;

      double z_;
      double z_dot_;
      double z_dot_ref_;

      Eigen::MatrixXd Jz_;
      Eigen::VectorXd err_;
};

}

#endif
