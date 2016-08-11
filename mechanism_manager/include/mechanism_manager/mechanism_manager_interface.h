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

enum scale_mode_t {HARD,POTENTIAL,SOFT};
class MechanismManagerServer;
class MechanismManager;

class MechanismManagerInterface
{

  public:
    MechanismManagerInterface();
    ~MechanismManagerInterface();

    // NOTE: We can not copy mechanism manager because of the internal thread and the mutex
    // We can explicit that thanks to C++11
    //MechanismManagerInterface( const MechanismManagerInterface& other ) = delete; // non construction-copyable
    //MechanismManagerInterface& operator=( const MechanismManagerInterface& ) = delete; // non copyable

    /// Real time loop
    void Update(const Eigen::VectorXd& robot_pose, const Eigen::VectorXd& robot_velocity, double dt, Eigen::VectorXd& f_out, const scale_mode_t scale_mode = SOFT);
    void Update(const double* robot_position_ptr, const double* robot_velocity_ptr, double dt, double* f_out_ptr, const scale_mode_t scale_mode = SOFT);

    /// Non real time services
    void InsertVM(std::string& model_name);
    void InsertVM(Eigen::MatrixXd& data);
    void InsertVM(double* data, const int n_rows);
    void InsertVM();
    void DeleteVM(const int idx);
    void SaveVM(const int idx);
    void SaveVM(const int idx, std::string& model_name);

    /// Stop the mechanisms
    void Stop();

    /// Check if the robot is on a guide
    bool OnVm();

    /// Sets
    inline bool SetCollision(bool collision_detected) {collision_detected_ = collision_detected;} // HACKY

    /// Gets
    inline int GetPositionDim() const {return position_dim_;}
    int GetNbVms();
    void GetVmPosition(const int idx, Eigen::VectorXd& position);
    void GetVmVelocity(const int idx, Eigen::VectorXd& velocity);
    void GetVmPosition(const int idx, double* const position_ptr); // Used by TAO
    void GetVmVelocity(const int idx, double* const velocity_ptr); // Used by TAO
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
