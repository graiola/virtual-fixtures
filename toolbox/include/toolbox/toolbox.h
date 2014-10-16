#ifndef TOOLBOX_H
#define TOOLBOX_H

////////// ROS
#include <ros/ros.h>

////////// Eigen
#include <eigen3/Eigen/Core>

////////// BOOST
#include <boost/bind.hpp>

namespace tool_box 
{

class AdaptiveGain
{
	public:
		
		AdaptiveGain(std::vector<double> gains_params)
		{
			assert(gains_params.size()==3);
			AdaptiveGain(gains_params[0],gains_params[1],gains_params[2]);
		}
		
		AdaptiveGain(double gain_at_zero, double gain_at_inf = 0.0, double zero_slope_error_value = 0.0)
		{
			if(gain_at_inf==0.0) // Constant gain
				b_ = 0.0;
			else // Adaptive gain
			{	
				// Checks
				assert(gain_at_inf > 0.0);
				assert(gain_at_zero > gain_at_inf);
				assert(zero_slope_error_value > 0.0);
				b_ = 6/zero_slope_error_value;
			}
			c_ = gain_at_inf;
			a_ = gain_at_zero - gain_at_inf;	
		}
		
		double ComputeGain(const double& error)
		{
			return a_ * std::exp(-b_ * std::abs(error)) + c_;
		}
		
	private:
		double a_;
		double b_;
		double c_;
};

class RosNode
{
	public:
		RosNode(std::string ros_node_name)
		{
		  
		  int argc = 1;
		  char* arg0 = strdup(ros_node_name.c_str());
		  char* argv[] = {arg0, 0};
		  ros::init(argc, argv, ros_node_name,ros::init_options::NoSigintHandler);
		  free (arg0);
		  if(ros::master::check()){
			  ros_nh_ptr_ = new ros::NodeHandle("");
		  }
		  else
		  {
		      std::string err("ERROR: roscore not found... Did you start the server?");
		      throw std::runtime_error(err);
		  }
		  
		}
		
		~RosNode(){if(ros_nh_ptr_!=NULL){ros_nh_ptr_->shutdown(); delete ros_nh_ptr_;}}
		
		const ros::NodeHandle& GetNode(){return *ros_nh_ptr_;}
		
	protected:
		ros::NodeHandle* ros_nh_ptr_;

};

//auto fn_half = std::bind (my_divide,_1,2);               // returns x/2
//std::cout << fn_half(10) << '\n'; 
/*RungeKutta(const Eigen::Ref<const Eigen::VectorXd>& state_init , Eigen::Ref<Eigen::VectorXd> state_out, double dt, boost::bind dyn_system)
{
  dyn_system()
}*/
  
	
}

#endif

