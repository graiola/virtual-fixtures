/**
 * @file   mechanism_manager_interface.h
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

#ifndef MECHANISM_MANAGER_INTERFACE_H
#define MECHANISM_MANAGER_INTERFACE_H

////////// Toolbox
#include <toolbox/toolbox.h>

////////// ROS
#include <ros/ros.h>
#include <ros/console.h>

////////// Eigen
#include <eigen3/Eigen/Core>

////////// BOOST
#include <boost/thread.hpp>


namespace mechanism_manager
{

enum scale_mode_t {HARD,SOFT};
class MechanismManagerServer;
class MechanismManager;
static bool default_threading_on = false;

class MechanismManagerInterface
{

  public:
    MechanismManagerInterface();
    ~MechanismManagerInterface();

    /// Real time loop
    void Update(const Eigen::VectorXd& robot_pose, const Eigen::VectorXd& robot_velocity, double dt, Eigen::VectorXd& f_out);
    void Update(const double* robot_position_ptr, const double* robot_velocity_ptr, double dt, double* f_out_ptr);

    /// Non real time async services
    /// threading enables the use of separate threads to ensure the real time
    void InsertVm(std::string& model_name, bool threading = default_threading_on);
    void InsertVm(Eigen::MatrixXd& data, bool threading = default_threading_on);
    void InsertVm(double* data, const int n_rows, bool threading = default_threading_on);
    void DeleteVm(const int idx, bool threading = default_threading_on);
    void UpdateVm(Eigen::MatrixXd& data, const int idx, bool threading = default_threading_on);
    void ClusterVm(Eigen::MatrixXd& data, bool threading = default_threading_on);
    void ClusterVm(double* data, const int n_rows, bool threading = default_threading_on);
    void SaveVm(const int idx, bool threading = default_threading_on);

    /// Non real time sync services
    void GetVmName(const int idx, std::string& name);
    void SetVmName(const int idx, std::string& name);
    void GetVmNames(std::vector<std::string>& names);
    void SetVmMode(const std::string mode);

    /// Stop the mechanisms
    void Stop();

    /// Check if the robot is on a guide
    bool OnVm();

    /// Sets
    inline bool SetCollision(bool collision_detected) {collision_detected_ = collision_detected;}

    /// Gets
    inline int GetPositionDim() const {return position_dim_;}
    int GetNbVms();
    void GetVmPosition(const int idx, Eigen::VectorXd& position);
    void GetVmVelocity(const int idx, Eigen::VectorXd& velocity);
    void GetVmPosition(const int idx, double* const position_ptr);
    void GetVmVelocity(const int idx, double* const velocity_ptr);
    double GetPhase(const int idx);
    double GetScale(const int idx);

    /// Sets
    void SetVmMode(const scale_mode_t mode);
    void SetMergeThreshold(int merge_th);

  protected:

    bool ReadConfig();
    //bool ExecuteService();

  private:

    Eigen::VectorXd tmp_eigen_vector_; // Used to convert std to eigen vector

    Eigen::VectorXd robot_position_;
    Eigen::VectorXd robot_velocity_;
    Eigen::VectorXd robot_orientation_;
    Eigen::VectorXd f_;

    int position_dim_;
    bool collision_detected_;

    // Mechanism Manager
    MechanismManager* mm_;

    // Thread stuff
    tool_box::AsyncThread* async_thread_;
    //tool_box::ThreadsPool* threads_pool_;
    //tool_box::AsyncThread* async_thread_insert_;
    //tool_box::AsyncThread* async_thread_delete_;
    //tool_box::AsyncThread* async_thread_save_;

    // Ros stuff
    tool_box::RosNode ros_node_;
    MechanismManagerServer* mm_server_;
};

}

#endif
