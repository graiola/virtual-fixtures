/**
 * @file   mechanism_manager_interface.cpp
 * @brief  External interface for the mechanism manager.
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

#include "mechanism_manager/mechanism_manager_interface.h"
#include "mechanism_manager/mechanism_manager_server.h"
#include "mechanism_manager/mechanism_manager.h"

namespace mechanism_manager
{

  using namespace tool_box;
  using namespace Eigen;

MechanismManagerInterface::MechanismManagerInterface(): mm_(NULL), mm_server_(NULL)
{
      //threads_pool_ = new ThreadsPool(4); // Create 4 workers

      //async_thread_insert_ = new AsyncThread();
      //async_thread_delete_ = new AsyncThread();
      //async_thread_save_ = new AsyncThread();
      async_thread_ = new AsyncThread();

      if(!ReadConfig())
      {
        PRINT_ERROR("MechanismManagerInterface: Can not read config file");
      }

      // Resize
      tmp_eigen_vector_.resize(position_dim_);
      robot_position_.resize(position_dim_);
      robot_velocity_.resize(position_dim_);
      f_.resize(position_dim_);

      // Clear
      tmp_eigen_vector_.fill(0.0);
      robot_position_.fill(0.0);
      robot_velocity_.fill(0.0);
      f_.fill(0.0);

      collision_detected_ = true; // Let's start not active

      try
      {
          ros_node_.Init(ROS_PKG_NAME);
          if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Count) )
             ros::console::notifyLoggerLevelsChanged();
          mm_server_ = new MechanismManagerServer(this,ros_node_.GetNode());
      }
      catch(const std::runtime_error& e)
      {
          ROS_ERROR("Failed to create the MechanismManagerServer: %s",e.what());
      }

      mm_ = new MechanismManager(position_dim_);
}

MechanismManagerInterface::~MechanismManagerInterface()
{
    //delete threads_pool_;
    //delete async_thread_insert_;
    //delete async_thread_delete_;
    //delete async_thread_save_;
    delete async_thread_;

    if(mm_server_!=NULL)
      delete mm_server_;

    delete mm_;
}

bool MechanismManagerInterface::ReadConfig()
{
    YAML::Node main_node = CreateYamlNodeFromPkgName(ROS_PKG_NAME);
    if (const YAML::Node& curr_node = main_node["mechanism_manager_interface"])
    {
        curr_node["position_dim"] >> position_dim_;
        assert(position_dim_ == 1 || position_dim_ == 2);

        return true;
    }
    else
        return false;
}

void MechanismManagerInterface::InsertVm(MatrixXd& data, bool threading)
{
    if(threading)
    {
        async_thread_->AddHandler(boost::bind(static_cast<void (MechanismManager::*)(const MatrixXd&)>(&MechanismManager::InsertVm), mm_, data));
        async_thread_->Trigger();
    }
    else
        mm_->InsertVm(data);
}

void MechanismManagerInterface::InsertVm(std::string& model_name, bool threading)
{
    if(threading)
    {
        async_thread_->AddHandler(boost::bind(static_cast<void (MechanismManager::*)(std::string&)>(&MechanismManager::InsertVm), mm_, model_name));
        async_thread_->Trigger();
    }
    else
        mm_->InsertVm(model_name);
}

void MechanismManagerInterface::InsertVm(double* data, const int n_rows, bool threading)
{
    if(threading)
    {
        async_thread_->AddHandler(boost::bind(&MechanismManager::InsertVm, mm_, data, n_rows));
        async_thread_->Trigger();
    }
    else
        mm_->InsertVm(data,n_rows);
}

void MechanismManagerInterface::UpdateVm(Eigen::MatrixXd& data, const int idx, bool threading)
{
    if(threading)
    {
        async_thread_->AddHandler(boost::bind(&MechanismManager::UpdateVm, mm_, data, idx));
        async_thread_->Trigger();
    }
    else
        mm_->UpdateVm(data,idx);
}

void MechanismManagerInterface::ClusterVm(Eigen::MatrixXd& data, bool threading)
{
    if(threading)
    {
        async_thread_->AddHandler(boost::bind(&MechanismManager::ClusterVm, mm_, data));
        async_thread_->Trigger();
    }
    else
        mm_->ClusterVm(data);
}

void MechanismManagerInterface::ClusterVm(double* data, const int n_rows, bool threading)
{
    if(threading)
    {
        async_thread_->AddHandler(boost::bind(&MechanismManager::ClusterVm, mm_, data, n_rows));
        async_thread_->Trigger();
    }
    else
        mm_->InsertVm(data,n_rows);
}

void MechanismManagerInterface::SaveVm(const int idx, bool threading)
{
    if(threading)
    {
        async_thread_->AddHandler(boost::bind(&MechanismManager::SaveVm, mm_, idx));
        async_thread_->Trigger();
    }
    else
        mm_->SaveVm(idx);
}

void MechanismManagerInterface::DeleteVm(const int idx, bool threading)
{
    if(threading)
    {
        async_thread_->AddHandler(boost::bind(&MechanismManager::DeleteVm, mm_, idx));
        async_thread_->Trigger();
    }
    else
        mm_->DeleteVm(idx);
}

void MechanismManagerInterface::SetVmMode(const scale_mode_t mode)
{
    mm_->SetMode(mode);
}

void MechanismManagerInterface::SetVmMode(const std::string mode)
{
    scale_mode_t enum_mode = SOFT;
    if(std::strcmp(mode.c_str(), "SOFT") == 0)
       enum_mode = SOFT;
    else if(std::strcmp(mode.c_str(), "HARD") == 0)
       enum_mode = HARD;

    mm_->SetVmMode(enum_mode);
}

void MechanismManagerInterface::SetMergeThreshold(int merge_th)
{
    mm_->SetMergeThreshold(merge_th);
}

void MechanismManagerInterface::GetMergeThreshold(int& merge_th)
{
    mm_->GetMergeThreshold(merge_th);
}

void MechanismManagerInterface::GetVmName(const int idx, std::string& name)
{
    mm_->GetVmName(idx,name);
}

void MechanismManagerInterface::GetVmNames(std::vector<std::string>& names)
{
    mm_->GetVmNames(names);
}

void MechanismManagerInterface::SetVmName(const int idx, std::string& name)
{
    mm_->SetVmName(idx,name);
}

void MechanismManagerInterface::Update(const double* robot_position_ptr, const double* robot_velocity_ptr, double dt, double* f_out_ptr)
{
    assert(dt > 0.0);

    robot_position_ = VectorXd::Map(robot_position_ptr, position_dim_);
    robot_velocity_ = VectorXd::Map(robot_velocity_ptr, position_dim_);

    mm_->Update(robot_position_,robot_velocity_,dt,f_);

    VectorXd::Map(f_out_ptr, position_dim_) = f_;
}

void MechanismManagerInterface::Update(const VectorXd& robot_position, const VectorXd& robot_velocity, double dt, VectorXd& f_out)
{
    assert(dt > 0.0);

    assert(robot_position.size() == position_dim_);
    assert(robot_velocity.size() == position_dim_);
    assert(f_out.size() == position_dim_);
    robot_position_ = robot_position;
    robot_velocity_ = robot_velocity;

    mm_->Update(robot_position_,robot_velocity_,dt,f_);

    f_out = f_;
}

/*void MechanismManagerInterface::CheckForGuideActivation(const int idx)
{
    //const double r = vm_vector_[idx]->getR();
    const double phase_dot = vm_vector_[idx]->getPhaseDot();
    const double phase_dot_ref = vm_vector_[idx]->getPhaseDotRef();
    vm_autom_[idx]->Step(phase_dot,phase_dot_ref,collision_detected_);
    if(vm_autom_[idx]->GetState())
        vm_vector_[idx]->setActive(true);
    else
        vm_vector_[idx]->setActive(false);

    //r_(idx) = r;
}*/

void MechanismManagerInterface::Stop()
{
    mm_->Stop();
}

void MechanismManagerInterface::GetVmPosition(const int idx, double* const position_ptr)
{
    tmp_eigen_vector_ = VectorXd::Map(position_ptr, position_dim_);
    mm_->GetVmPosition(idx,tmp_eigen_vector_);
    VectorXd::Map(position_ptr, position_dim_) = tmp_eigen_vector_;
}

void MechanismManagerInterface::GetVmVelocity(const int idx, double* const velocity_ptr)
{
    tmp_eigen_vector_ = VectorXd::Map(velocity_ptr, position_dim_);
    mm_->GetVmVelocity(idx,tmp_eigen_vector_);
    VectorXd::Map(velocity_ptr, position_dim_) = tmp_eigen_vector_;
}

void MechanismManagerInterface::GetVmPosition(const int idx, Eigen::VectorXd& position)
{
    mm_->GetVmPosition(idx,position);
}

void MechanismManagerInterface::GetVmVelocity(const int idx, Eigen::VectorXd& velocity)
{
    mm_->GetVmVelocity(idx,velocity);
}

double MechanismManagerInterface::GetPhase(const int idx)
{
    return mm_->GetPhase(idx);
}
double MechanismManagerInterface::GetScale(const int idx)
{
    return mm_->GetScale(idx);
}

int MechanismManagerInterface::GetNbVms()
{
    return mm_->GetNbVms();
}

bool MechanismManagerInterface::OnVm()
{
    return mm_->OnVm();
}

} // namespace
