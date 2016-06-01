#ifndef MECHANISM_MANAGER_H
#define MECHANISM_MANAGER_H

//#define USE_ROS_RT_PUBLISHER

////////// Toolbox
#include <toolbox/toolbox.h>
#include <toolbox/filters/filters.h>

#ifdef INCLUDE_ROS_CODE
    ////////// ROS
    #include <ros/ros.h>
    #include <ros/package.h>
#endif

////////// YAML-CPP
#include <yaml-cpp/yaml.h>

////////// Eigen
#include <eigen3/Eigen/Core>

////////// VIRTUAL_MECHANISM
#include <virtual_mechanism/virtual_mechanism_interface.h>
#include <virtual_mechanism/virtual_mechanism_gmr.h>
#include <virtual_mechanism/virtual_mechanism_spline.h>

////////// BOOST
#include <boost/thread.hpp>

namespace mechanism_manager
{

typedef boost::mutex mutex_t;
typedef virtual_mechanism_interface::VirtualMechanismInterface vm_t;
enum prob_mode_t {HARD,POTENTIAL,SOFT};

template <typename _T >
void operator >>(const YAML::Node& input, _T& value) {
      value = input.as<_T>();
}
template <typename _T >
void operator >> (const YAML::Node &node, std::vector<_T> & v)
{
      for(unsigned i = 0; i < node.size(); i++)
	      v.push_back(node[i].as<_T>());
}

class VirtualMechanismAutom
{
public:
    VirtualMechanismAutom(const double phase_dot_preauto_th, const double phase_dot_th, const double phase_ddot_th);

    void Step(const double phase_dot,const double phase_dot_ref, const double r);
    bool GetState();

private:
    enum state_t {MANUAL,PREAUTO,AUTO};
    double phase_dot_preauto_th_;
    double phase_dot_th_;
    double r_th_;
    state_t state_;
};

/*struct vm_struct
{
    Eigen::VectorXd state;
    Eigen::VectorXd state_dot;
    vm_t* vm;
};*/

class MechanismManager
{ 
  public:
    MechanismManager();
    ~MechanismManager();

    // NOTE: We can not copy mechanism manager because of the internal thread and the mutex
    // We can explicit that thanks to C++11
    MechanismManager( const MechanismManager& other ) = delete; // non construction-copyable
    MechanismManager& operator=( const MechanismManager& ) = delete; // non copyable
  
    // Loop Update Interfaces
    void Update(const Eigen::VectorXd& robot_pose, const Eigen::VectorXd& robot_velocity, double dt, Eigen::VectorXd& f_out, const prob_mode_t prob_mode = SOFT);
    void Update(const double* robot_position_ptr, const double* robot_velocity_ptr, double dt, double* f_out_ptr, const prob_mode_t prob_mode = SOFT);

    // Mechanism Manager external interface
    void InsertVM(std::string model_name);
    void DeleteVM(const int idx);
    void Stop();

    // Gets
    inline int GetPositionDim() const {return position_dim_;}
    inline int GetNbVms() const {return vm_vector_.size();}
    void GetVmPosition(const int idx, Eigen::VectorXd& position);
    void GetVmVelocity(const int idx, Eigen::VectorXd& velocity);
    void GetVmPosition(const int idx, double* const position_ptr);
    void GetVmVelocity(const int idx, double* const velocity_ptr);
    double GetPhase(const int idx);
    double GetScale(const int idx);

  protected:
    void Update(const prob_mode_t prob_mode);
    bool ReadConfig(std::string file_path);
    void Delete(const int idx, Eigen::VectorXd& vect);
    void PushBack(const double value, Eigen::VectorXd& vect);


  private:   

    void InsertVM_no_rt(std::string& model_name); // No Real time
    void DeleteVM_no_rt(const int& idx); // No Real time

    //enum prob_mode_t {HARD,POTENTIAL,SOFT};
    //prob_mode_t prob_mode_;
    
    std::string pkg_path_;
    
    long long loopCnt;

    //std::vector<bool> activated_;
    //std::vector<bool> active_guide_;

    Eigen::VectorXd tmp_eigen_vector_; // Used to convert std to eigen vector


    Eigen::VectorXd f_pos_;
    Eigen::VectorXd f_pos_prev_;
    Eigen::VectorXd f_ori_;
    Eigen::VectorXd robot_position_;
    Eigen::VectorXd robot_velocity_;
    Eigen::VectorXd robot_orientation_;
    Eigen::VectorXd scales_;

    Eigen::VectorXd phase_;
    Eigen::VectorXd phase_dot_;
    Eigen::VectorXd phase_ddot_;
    Eigen::VectorXd phase_ref_;
    Eigen::VectorXd phase_dot_ref_;
    Eigen::VectorXd phase_ddot_ref_;
    Eigen::VectorXd fade_;
    Eigen::VectorXd r_;

    int position_dim_;
    int orientation_dim_;
    int n_samples_filter_;
    double sum_;

    double dt_;
    double escape_factor_;

    double phase_dot_th_;
    double r_th_;
    double pre_auto_th_;

    std::vector<Eigen::VectorXd> vm_state_;
    std::vector<Eigen::VectorXd> vm_state_dot_;
    std::vector<vm_t*> vm_vector_;

    //std::vector<vm_struct> vm_vector_;
    bool second_order_;
    bool use_weighted_dist_;
    bool use_active_guide_;
    double execution_time_;
    double K_;
    double B_;
    double Kf_;
    double Bf_;
    double inertia_;
    double Kr_;
    double fade_gain_;
    bool use_orientation_;

    std::vector<filters::M3DFilter* > filter_phase_dot_;
    std::vector<filters::M3DFilter* > filter_phase_ddot_;
    std::vector<VirtualMechanismAutom* > vm_autom_;

    // Thread stuff
    boost::thread thread_insert_;
    boost::thread thread_delete_;
    mutex_t mtx_;


#ifdef INCLUDE_ROS_CODE
    #ifdef USE_ROS_RT_PUBLISHER
        tool_box::RosNode ros_node_;
        tool_box::RealTimePublishers<tool_box::RealTimePublisherVector> rt_publishers_vector_;
        //tool_box::RealTimePublishers<tool_box::RealTimePublisherPath> rt_publishers_path_;
    #endif
#endif
    
};

}

#endif
