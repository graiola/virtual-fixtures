#ifndef MECHANISM_MANAGER_H
#define MECHANISM_MANAGER_H

//#define USE_ROS_RT_PUBLISHER

////////// Toolbox
#include "toolbox/toolbox.h"

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

namespace mechanism_manager
{

//typedef virtual_mechanism_gmr::VirtualMechanismGmrSplined<virtual_mechanism_interface::VirtualMechanismInterfaceFirstOrder> vm_t;
typedef virtual_mechanism_interface::VirtualMechanismInterface vm_t;

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
  
class MechanismManager
{
  
  public:
    MechanismManager();
    ~MechanismManager();
  
    void Update(const Eigen::VectorXd& robot_pose, const Eigen::VectorXd& robot_velocity, double dt, Eigen::VectorXd& f_out);
    void Update(const Eigen::VectorXd& robot_pose, const Eigen::VectorXd& robot_velocity, double dt, Eigen::VectorXd& f_out, const bool user_force_applied);
    void Update(const double* robot_position_ptr, const double* robot_velocity_ptr, double dt, double* f_out_ptr);
    void Update(const double* robot_position_ptr, const double* robot_velocity_ptr, double dt, double* f_out_ptr, const bool force_applied);

    //void Update(const Eigen::VectorXd& robot_pose, const Eigen::VectorXd& robot_velocity, double dt, Eigen::VectorXd& f_out, bool force_applied, bool move_forward);
    inline double GetPhase(const int idx) {assert(idx <= vm_vector_.size()); return vm_vector_[idx]->getPhase();}
    inline double GetScale(const int idx) {assert(idx <= scales_.size()); return scales_(idx);}
    inline void GetVmPosition(const int idx, Eigen::VectorXd& position) {assert(idx <= vm_vector_.size()); vm_vector_[idx]->getState(position);}
    inline void GetVmVelocity(const int idx, Eigen::VectorXd& velocity) {assert(idx <= vm_vector_.size()); vm_vector_[idx]->getStateDot(velocity);}
    void GetVmPosition(const int idx, const double* position_ptr);
    void GetVmVelocity(const int idx, const double* velocity_ptr);
    int GetPositionDim() const {return position_dim_;}
    inline int GetNbVms() {return vm_nb_;}
    //void MoveForward();
    //void MoveBackward();
    
    
  protected:
    void Update();
    bool ReadConfig(std::string file_path);
   
  private:
    
    void UpdateTrackingReference(const Eigen::VectorXd& robot_position); // To update the reference for the head tracking
    
    enum prob_mode_t {HARD,POTENTIAL,SOFT,ESCAPE};
    prob_mode_t prob_mode_;
    
    std::string pkg_path_;
    
    //boost::shared_ptr<fa_t> fa_shr_ptr_; // Function Approximator
    //std::vector<boost::shared_ptr<fa_t> > fa_vector_;
    
    long long loopCnt;

    bool user_force_applied_;
    bool use_orientation_;
    std::vector<bool> active_guide_;
    Eigen::VectorXd tmp_eigen_vector_;
    Eigen::VectorXd scales_;
    Eigen::VectorXd escape_factors_;
    Eigen::VectorXd escape_field_;
    Eigen::VectorXd escape_field_compare_;
    //Eigen::VectorXd Kf_;
    Eigen::VectorXd phase_dot_;
    //Eigen::VectorXd phase_ddot_;
    Eigen::VectorXd robot_position_;
    Eigen::VectorXd robot_velocity_;
    Eigen::VectorXd robot_orientation_;
    Eigen::VectorXd orientation_error_;
    Eigen::VectorXd cross_prod_;
    Eigen::VectorXd prev_orientation_error_;
    Eigen::VectorXd orientation_integral_;
    Eigen::VectorXd orientation_derivative_;
    Eigen::VectorXd f_pos_;
    Eigen::VectorXd f_ori_;
    Eigen::VectorXd f_rob_;
    Eigen::VectorXd t_versor_;
    Eigen::VectorXd f_rob_t_;
    Eigen::VectorXd f_rob_n_;
    //Eigen::VectorXd prob_;
    int vm_nb_;
    int position_dim_;
    int orientation_dim_;
    double sum_;
    double curr_norm_factor_;
    double scale_threshold_;
    double dt_;
    double escape_factor_;
    double f_norm_;
    Eigen::VectorXd phase_;
    std::vector<Eigen::VectorXd> vm_state_;
    std::vector<Eigen::VectorXd> vm_state_dot_;
    std::vector<Eigen::VectorXd> vm_jacobian_;
    std::vector<Eigen::VectorXd> vm_quat_;
    std::vector<std::vector<double> > quat_start_;
    std::vector<std::vector<double> > quat_end_;
    
    // FIXME Put them in a struct...?
    std::vector<vm_t*> vm_vector_; // TODO move chose of template to ReadConfig
    std::vector<bool> use_weighted_dist_;
    std::vector<bool> use_active_guide_;
    //std::vector<boost::circular_buffer<double> > activation_values_;
    //std::vector<Eigen::VectorXd> vm_state_;
    //std::vector<Eigen::VectorXd> vm_state_dot_;
    //std::vector<Eigen::VectorXd> vm_kernel_;

#ifdef INCLUDE_ROS_CODE
    #ifdef USE_ROS_RT_PUBLISHER
        tool_box::RosNode ros_node_;
        tool_box::RealTimePublishers<tool_box::RealTimePublisherJoints> rt_publishers_values_;
        //tool_box::RealTimePublishers<tool_box::RealTimePublisherWrench> rt_publishers_wrench_;
        tool_box::RealTimePublishers<tool_box::RealTimePublisherPath> rt_publishers_path_;
    #endif
#endif
    
};

}

#endif
