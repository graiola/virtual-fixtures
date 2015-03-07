#ifndef MECHANISM_MANAGER_H
#define MECHANISM_MANAGER_H

//#define USE_ROS_RT_PUBLISHER

////////// Toolbox
#include <toolbox/toolbox.h>

////////// ROS
#include <ros/ros.h>
#include <ros/package.h>

////////// YAML-CPP
#include <yaml-cpp/yaml.h>

////////// Eigen
#include <eigen3/Eigen/Core>

////////// VIRTUAL_MECHANISM
#include <virtual_mechanism/virtual_mechanism_gmr.h>

////////// Function Approximator
#include <functionapproximators/FunctionApproximatorGMR.hpp>
#include <functionapproximators/MetaParametersGMR.hpp>
#include <functionapproximators/ModelParametersGMR.hpp>

namespace mechanism_manager
{

typedef DmpBbo::FunctionApproximatorGMR fa_t;
  
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
  
    void Update(const Eigen::Ref<const Eigen::VectorXd>& robot_position, const Eigen::Ref<const Eigen::VectorXd>& robot_velocity, double dt, Eigen::Ref<Eigen::VectorXd> f_out);
    
  protected:
    
    bool ReadConfig(std::string file_path);
   
  private:
    enum prob_mode_t {CONDITIONAL,PRIORS,MIX};
    prob_mode_t prob_mode_;
    
    std::string pkg_path_;
    
    //boost::shared_ptr<fa_t> fa_shr_ptr_; // Function Approximator
    //std::vector<boost::shared_ptr<fa_t> > fa_vector_;
    
    int vm_nb_;
    int dim_;
    double sum_;
    Eigen::VectorXd scales_;
    Eigen::VectorXd phase_;
    Eigen::VectorXd robot_position_;
    std::vector<Eigen::VectorXd> vm_state_;
    std::vector<Eigen::VectorXd> vm_state_dot_;
    
    // FIXME Put them in a struct...?
    std::vector<virtual_mechanism_gmr::VirtualMechanismGmr*> vm_vector_; // TODO to template order
    //std::vector<Eigen::VectorXd> vm_state_;
    //std::vector<Eigen::VectorXd> vm_state_dot_;
    //std::vector<Eigen::VectorXd> vm_kernel_;
    
    //std::vector<bool> use_weighted_dist_;
    //std::vector<bool> adapt_gains_;
    
    
    
#ifdef USE_ROS_RT_PUBLISHER
    tool_box::RosNode ros_node_;
    tool_box::RealTimePublishers<tool_box::RealTimePublisherJoints> rt_publishers_values_;
    tool_box::RealTimePublishers<tool_box::RealTimePublisherPath> rt_publishers_path_;
#endif
    
};

}

#endif