#ifndef MECHANISM_MANAGER_H
#define MECHANISM_MANAGER_H

//#define USE_ROS_RT_PUBLISHER

////////// Toolbox
#include "toolbox/toolbox.h"
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
  

class VirtualMechanismAutom
{

public:
    VirtualMechanismAutom(const double phase_dot_preauto_th, const double phase_dot_th, const double phase_ddot_th);
    //~VirtualMechanismAutom();

    void Step(const double phase_dot,const double phase_dot_ref, const double r);
    bool GetState();

private:
    enum state_t {MANUAL,PREAUTO,AUTO};
    double phase_dot_preauto_th_;
    double phase_dot_th_;
    double r_th_;
    state_t state_;
    long long loopcnt_;
};

class MechanismManager
{
  
  public:
    MechanismManager();
    ~MechanismManager();
  

    // Loop Update
    void Update(const Eigen::VectorXd& robot_pose, const Eigen::VectorXd& robot_velocity, double dt, Eigen::VectorXd& f_out);
    void Update(const double* robot_position_ptr, const double* robot_velocity_ptr, double dt, double* f_out_ptr);



    inline double GetPhase(const int idx) {assert(idx <= vm_vector_.size()); return vm_vector_[idx]->getPhase();}
    inline double GetScale(const int idx) {assert(idx <= scales_.size()); return scales_(idx);}
    inline void GetVmPosition(const int idx, Eigen::VectorXd& position) {assert(idx <= vm_vector_.size()); vm_vector_[idx]->getState(position);}
    inline void GetVmVelocity(const int idx, Eigen::VectorXd& velocity) {assert(idx <= vm_vector_.size()); vm_vector_[idx]->getStateDot(velocity);}
    void GetVmPosition(const int idx, double* const position_ptr);
    void GetVmVelocity(const int idx, double* const velocity_ptr);
    int GetPositionDim() const {return position_dim_;}
    inline int GetNbVms() {return vm_nb_;}
    void Stop();
    //void MoveForward();
    //void MoveBackward();
    
    
  protected:
    void Update();
    bool ReadConfig(std::string file_path);
   
  private:
    
    void UpdateTrackingReference(const Eigen::VectorXd& robot_position); // To update the reference for the head tracking
    
    enum prob_mode_t {HARD,POTENTIAL,SOFT,ESCAPE};
    enum mechanism_t {GMM,GMM_NORMALIZED,SPLINE};
    prob_mode_t prob_mode_;
    mechanism_t mechanism_type_;
    
    std::string pkg_path_;
    
    //boost::shared_ptr<fa_t> fa_shr_ptr_; // Function Approximator
    //std::vector<boost::shared_ptr<fa_t> > fa_vector_;
    
    long long loopCnt;

    std::vector<bool> activated_;
    bool use_orientation_;
    std::vector<bool> active_guide_;
    Eigen::VectorXd tmp_eigen_vector_;
    Eigen::VectorXd scales_;
    Eigen::VectorXd escape_factors_;
    Eigen::VectorXd escape_field_;
    //Eigen::VectorXd escape_field_compare_;
    //Eigen::VectorXd Kf_;
    Eigen::VectorXd phase_dot_;
    Eigen::VectorXd phase_ddot_;
    Eigen::VectorXd robot_position_;
    Eigen::VectorXd robot_velocity_;
    Eigen::VectorXd robot_orientation_;
    Eigen::VectorXd orientation_error_;
    Eigen::VectorXd cross_prod_;
    /*Eigen::VectorXd prev_orientation_error_;
    Eigen::VectorXd orientation_integral_;
    Eigen::VectorXd orientation_derivative_;*/
    Eigen::VectorXd f_pos_;
    Eigen::VectorXd f_ori_;
    Eigen::VectorXd f_rob_;
    Eigen::VectorXd t_versor_;
    Eigen::VectorXd f_rob_t_;
    Eigen::VectorXd f_rob_n_;
    Eigen::VectorXd phase_dot_filt_;
    Eigen::VectorXd phase_ddot_filt_;
    Eigen::VectorXd phase_ref_;
    Eigen::VectorXd phase_dot_ref_;
    Eigen::VectorXd phase_ddot_ref_;
    Eigen::VectorXd phase_dot_ref_upper_;
    Eigen::VectorXd phase_dot_ref_lower_;
    Eigen::VectorXd fade_;
    Eigen::VectorXd r_;
    //Eigen::VectorXd p_dot_integrated_;
    //Eigen::VectorXd prob_;
    Eigen::VectorXd torque_;
    int vm_nb_;
    int position_dim_;
    int orientation_dim_;
    int n_samples_filter_;
    double sum_;
    double curr_norm_factor_;
    double scale_threshold_;
    double dt_;
    double escape_factor_;
    double f_norm_;
    //double range_;
    double phase_dot_th_;
    double r_th_;
    double pre_auto_th_;
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
    std::vector<double> execution_time_;
    std::vector<bool> use_active_guide_;
    //std::vector<boost::circular_buffer<double> > activation_values_;
    //std::vector<Eigen::VectorXd> vm_state_;
    //std::vector<Eigen::VectorXd> vm_state_dot_;
    //std::vector<Eigen::VectorXd> vm_kernel_;

    std::vector<filters::M3DFilter* > filter_phase_dot_;
    std::vector<filters::M3DFilter* > filter_phase_ddot_;
    std::vector<VirtualMechanismAutom* > vm_autom_;


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
