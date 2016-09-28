/**
 * @file   mechanism_manager.h
 * @brief  Manager of guides.
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

#ifndef MECHANISM_MANAGER_H
#define MECHANISM_MANAGER_H

////////// Toolbox
#include <toolbox/toolbox.h>
#include <toolbox/filters/filters.h>

////////// ROS
#include <ros/ros.h>

////////// Eigen
#include <eigen3/Eigen/Core>

////////// VIRTUAL_MECHANISM
#include <virtual_mechanism/virtual_mechanism_factory.h>

///////// MECHANISM_MANAGER
#include "mechanism_manager/mechanism_manager_interface.h"

namespace mechanism_manager
{

typedef boost::recursive_mutex mutex_t;
typedef virtual_mechanism::VirtualMechanismInterface vm_t;

struct GuideStruct
{
  std::string name;
  double scale;
  double scale_hard;
  double scale_t;
  boost::shared_ptr<vm_t> guide;
  boost::shared_ptr<tool_box::DynSystemFirstOrder> fade;
};

class MechanismManager
{

  public:
    MechanismManager(int position_dim);
    ~MechanismManager();

    // NOTE: We can not copy mechanism manager because of the internal thread and the mutex
    // We can explicit that thanks to C++11
    //MechanismManager( const MechanismManager& other ) = delete; // non construction-copyable
    //MechanismManager& operator=( const MechanismManager& ) = delete; // non copyable
  
    /// Loop Update Interface
    void Update(const Eigen::VectorXd& robot_position, const Eigen::VectorXd& robot_velocity, double dt, Eigen::VectorXd& f_out);

    /// Non Real time methods, to be launched in seprated threads
    void InsertVm(std::string& model_name);
    void InsertVm(const Eigen::MatrixXd& data);
    void InsertVm(double* data, const int n_rows);
    void DeleteVm(const int idx);
    void UpdateVm(Eigen::MatrixXd& data, const int idx);
    void ClusterVm(Eigen::MatrixXd& data);
    void ClusterVm(double* data, const int n_rows);
    void SaveVm(const int idx);
    void GetVmName(const int idx, std::string& name);
    void SetVmName(const int idx, std::string& name);
    void GetVmNames(std::vector<std::string>& names);
    void SetVmMode(const scale_mode_t mode);
    scale_mode_t& GetVmMode();
    void SetMergeThreshold(double merge_th);
    void GetMergeThreshold(double& merge_th);


    /// Real time methods, they can be called in a real time loop
    inline int GetPositionDim() const {return position_dim_;}
    int GetNbVms();
    void GetVmPosition(const int idx, Eigen::VectorXd& position);
    void GetVmVelocity(const int idx, Eigen::VectorXd& velocity);
    double GetPhase(const int idx);
    double GetScale(const int idx);
    void SetMode(const scale_mode_t mode);
    void Stop();
    bool OnVm();
    void SetCollisionDetected(const bool collision);

  protected:

    bool ReadConfig();
    void AddNewVm(vm_t* const vm_tmp_ptr, std::string& name);
    bool CheckForNamesCollision(const std::string& name);

    scale_mode_t scale_mode_;

    double merge_th_;

  private:   
    
    long long loopCnt;

    virtual_mechanism::VirtualMechanismFactory vm_factory_;

    /// For computations
    Eigen::VectorXd f_K_;
    Eigen::VectorXd f_B_;
    Eigen::VectorXd f_vm_;
    Eigen::VectorXd err_pos_;
    Eigen::VectorXd err_vel_;
    Eigen::VectorXd robot_position_;
    Eigen::VectorXd robot_velocity_;

    int position_dim_;

    double escape_factor_;

    std::string pkg_path_;
    int guide_unique_id_; // Incremental id

    /// Double buffer http://gameprogrammingpatterns.com/double-buffer.html
    /// Mechanism used: Page-flipping
    std::vector<GuideStruct> vm_buffers_[2];
    std::atomic<int> rt_idx_; // atom
    std::atomic<int> no_rt_idx_; // atom
    mutex_t mtx_;
};

}

#endif
