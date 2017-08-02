/**
 * @file   virtual_mechanism_spline.cpp
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

#include "virtual_mechanism/virtual_mechanism_spline.h"

using namespace std;
using namespace Eigen;
using namespace tool_box;
using namespace tk;

namespace virtual_mechanism
{

template <typename VM_t>
VirtualMechanismSpline<VM_t>::VirtualMechanismSpline(const Eigen::MatrixXd& data):
  VM_t()
{

  int n_points = data.rows();
  int state_dim = data.cols();
  int phase_dim = 1;

  VM_t::Resize(state_dim,phase_dim);
  err_.resize(state_dim);

  Eigen::MatrixXd arclength;
  ComputeArcLength(data,arclength);

  // Convert Eigen to std::vector
  vector<double> tmp_arclength(n_points,0.0);
  vector<vector<double> > tmp_xyz(state_dim, std::vector<double>(n_points));
  splines_xyz_.resize(state_dim);
  for(int i=0;i<n_points;i++)
  {
    tmp_arclength[i] = arclength(i);
    for(int j=0;j<state_dim;j++)
      tmp_xyz[j][i] = data(i,j);
  }

  for(int i=0;i<state_dim;i++)
    splines_xyz_[i].set_points(tmp_arclength,tmp_xyz[i]);

  data_ = data;
}

template <typename VM_t>
void VirtualMechanismSpline<VM_t>::UpdateJacobian()
{
  for(int i=0;i<VM_t::state_dim_;i++)
  {
    VM_t::J_transp_(0,i) = splines_xyz_[i].compute_derivate(VM_t::phase_(0));
  }
  VM_t::J_ = VM_t::J_transp_.transpose();
}

template<typename VM_t>
void VirtualMechanismSpline<VM_t>::UpdateState()
{
  for(int i=0;i<VM_t::state_dim_;i++)
    VM_t::state_(i) = splines_xyz_[i](VM_t::phase_(0));
}

template<typename VM_t>
void VirtualMechanismSpline<VM_t>::UpdateStateDot()
{
  VM_t::state_dot_.noalias() = VM_t::J_ * VM_t::phase_dot_;
}

template <class VM_t>
bool VirtualMechanismSpline<VM_t>::SaveModelToFile(const string model_path)
{
  //TODO
  return true;
}

template<class VM_t>
double VirtualMechanismSpline<VM_t>::getDistance(const VectorXd& pos)
{
  err_ = pos - VM_t::state_;
  return err_.norm();
}

template<class VM_t>
double VirtualMechanismSpline<VM_t>::getScale(const VectorXd& pos, const double convergence_factor)
{
  return std::exp(-convergence_factor*getDistance(pos));
}

template<class VM_t>
void VirtualMechanismSpline<VM_t>::Visualize(const string frame, const string node_name)
{
    VM_t::marker_.header.frame_id = frame;
    VM_t::marker_.action = visualization_msgs::Marker::ADD;
    VM_t::marker_.pose.orientation.w = 1.0;

    //VM_t::marker_.lifetime = ros::Duration();
    //VM_t::marker_.frame_locked = true;

    VM_t::marker_.id = 1;

    VM_t::marker_.type = visualization_msgs::Marker::LINE_STRIP;

    // VM_t::marker_/LINE_LIST markers use only the x component of scale, for the line width
    VM_t::marker_.scale.x = 0.01;

    // Line strip is blue
    VM_t::marker_.color.b = 1.0;
    VM_t::marker_.color.a = 1.0;

    geometry_msgs::Point p;
    for(int i = 0; i < data_.rows(); i++)
    {
      p.x = data_(i,0);
      p.y = data_(i,1);
      p.z = data_(i,2);
      VM_t::marker_.points.push_back(p);
    }
}

// Explicitly instantiate the templates, and its member definitions
template class VirtualMechanismSpline<VirtualMechanismInterfaceFirstOrder>;
template class VirtualMechanismSpline<VirtualMechanismInterfaceSecondOrder>;
}
