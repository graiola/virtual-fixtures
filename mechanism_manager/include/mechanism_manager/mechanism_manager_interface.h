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

class MechanismManagerInterface
{

  public:
    MechanismManagerInterface();
    ~MechanismManagerInterface();

    /// Real time loop
    void Update(const Eigen::VectorXd& robot_pose, const Eigen::VectorXd& robot_velocity, double dt, Eigen::VectorXd& f_out, const scale_mode_t scale_mode = SOFT);
    void Update(const double* robot_position_ptr, const double* robot_velocity_ptr, double dt, double* f_out_ptr, const scale_mode_t scale_mode = SOFT);

    /// Non real time services
    void InsertVm(std::string& model_name);
    void InsertVm(Eigen::MatrixXd& data);
    void InsertVm(double* data, const int n_rows);
    void DeleteVm(const int idx);
    void UpdateVm(Eigen::MatrixXd& data, const int idx);
    void ClusterVm(Eigen::MatrixXd& data);
    void SaveVm(const int idx);
    void GetVmName(const int idx, std::string& name);
    void SetVmName(const int idx, std::string& name);
    void GetVmNames(std::vector<std::string>& names);

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

  protected:

    bool ReadConfig();

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
    //tool_box::ThreadsPool* threads_pool_;
    tool_box::AsyncThread* async_thread_insert_;
    tool_box::AsyncThread* async_thread_delete_;
    tool_box::AsyncThread* async_thread_save_;

    // Ros stuff
    tool_box::RosNode ros_node_;
    MechanismManagerServer* mm_server_;
#ifdef USE_ROS_RT_PUBLISHER
    tool_box::RealTimePublishers<tool_box::RealTimePublisherVector> rt_publishers_vector_;
#endif
};

}

#endif
