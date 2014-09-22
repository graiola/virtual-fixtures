#ifndef TOOLBOX_H
#define TOOLBOX_H

////////// ROS
#include <ros/ros.h>

namespace tool_box {


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

  
	
}

#endif

