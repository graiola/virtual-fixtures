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

// It is here just for the prob enum
#include "mechanism_manager/mechanism_manager_interface.h"

namespace mechanism_manager
{

typedef boost::recursive_mutex mutex_t;
typedef virtual_mechanism::VirtualMechanismInterface vm_t;

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
    void Update(const Eigen::VectorXd& robot_position, const Eigen::VectorXd& robot_velocity, double dt, Eigen::VectorXd& f_out, const scale_mode_t scale_mode);

    /// Mechanism Manager external interface
    void InsertVM(std::string& model_name);
    void InsertVM();
    void InsertVM(const Eigen::MatrixXd& data);
    void DeleteVM(const int idx);
    void UpdateVM(Eigen::MatrixXd& data, const int idx);
    void ClusterVM(Eigen::MatrixXd& data);
    void SaveVM(const int idx);
    void SaveVM(const int idx, std::string& model_name);
    void Stop();
    bool OnVm();

    /// Gets
    inline int GetPositionDim() const {return position_dim_;}
    int GetNbVms();
    void GetVmPosition(const int idx, Eigen::VectorXd& position);
    void GetVmVelocity(const int idx, Eigen::VectorXd& velocity);
    double GetPhase(const int idx);
    double GetScale(const int idx);

  protected:

    bool ReadConfig();
    void InitGuide(vm_t* const vm_tmp_ptr);

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
    Eigen::VectorXd scales_;
    Eigen::VectorXd scales_hard_;
    Eigen::VectorXd scales_soft_;
    Eigen::VectorXd scales_t_;

    /// For plots
    Eigen::VectorXd phase_;
    Eigen::VectorXd phase_dot_;
    Eigen::VectorXd phase_ddot_;
    Eigen::VectorXd phase_ref_;
    Eigen::VectorXd phase_dot_ref_;
    Eigen::VectorXd phase_ddot_ref_;
    Eigen::VectorXd fade_;

    int position_dim_;

    double escape_factor_;

    std::string pkg_path_;

    std::vector<tool_box::DynSystemFirstOrder> vm_fades_;
    std::vector<vm_t*> vm_vector_;

    /// Mutex
    mutex_t mtx_;

    /// Cached values
    Eigen::VectorXd f_prev_;
    boost::atomic<bool> on_guide_prev_; // atom
    boost::atomic<int> nb_vm_prev_; // atom
};

}

#endif
