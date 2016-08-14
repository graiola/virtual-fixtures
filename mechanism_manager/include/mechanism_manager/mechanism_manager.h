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
  tool_box::DynSystemFirstOrder fade;
  double scale;
  double scale_hard;
  double scale_t;
  boost::shared_ptr<vm_t> guide;
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
    void Update(const Eigen::VectorXd& robot_position, const Eigen::VectorXd& robot_velocity, double dt, Eigen::VectorXd& f_out, const scale_mode_t scale_mode);

    /// Non Real time methods, to be launched in seprated threads
    void InsertVM(std::string& model_name);
    void InsertVM(const Eigen::MatrixXd& data);
    void DeleteVM(const int idx);
    void UpdateVM(Eigen::MatrixXd& data, const int idx);
    void ClusterVM(Eigen::MatrixXd& data);
    void SaveVM(const int idx);
    void GetVmName(const int idx, std::string& name);
    void GetVmNames(std::vector<std::string>& names);


    /// Real time methods, they can be called in a real time loop
    inline int GetPositionDim() const {return position_dim_;}
    int GetNbVms();
    void GetVmPosition(const int idx, Eigen::VectorXd& position);
    void GetVmVelocity(const int idx, Eigen::VectorXd& velocity);
    double GetPhase(const int idx);
    double GetScale(const int idx);
    void Stop();
    bool OnVm();

  protected:

    bool ReadConfig();
    void ExpandVectors(vm_t* const vm_tmp_ptr);

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

    /// Double buffer http://gameprogrammingpatterns.com/double-buffer.html
    std::vector<GuideStruct> vm_buffers_[2];
    boost::atomic<int> rt_idx_; // atom
    boost::atomic<int> no_rt_idx_; // atom
    mutex_t mtx_;

};

}

#endif
