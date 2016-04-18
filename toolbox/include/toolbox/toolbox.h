#ifndef TOOLBOX_H
#define TOOLBOX_H

////////// STD
#include <iostream>
#include <fstream>
#include <iterator>

#ifdef INCLUDE_ROS_CODE
	////////// ROS
	#include <ros/ros.h>
	#ifdef USE_ROS_RT_PUBLISHER
	//#ifdef INCLUDE_ROS_RT_PUBLISHER
          //#define USE_ROS_RT_PUBLISHER
	  #include <sensor_msgs/JointState.h>
	  #include <geometry_msgs/PoseStamped.h>
	  #include <geometry_msgs/WrenchStamped.h>
	  #include <visualization_msgs/Marker.h>
	  #include <nav_msgs/Path.h>
	  #include <realtime_tools/realtime_publisher.h>
	#endif
#endif

////////// Eigen
#include <eigen3/Eigen/Core>

////////// BOOST
#include <boost/bind.hpp>

namespace tool_box 
{

const std::string ref_frame_name = "map"; // FIXME

class MinJerk
{
	public:
		MinJerk():x(0),v(0),p(0),j(0){}
		inline double GetX() const {return x;}
		inline double GetXDot() const {return v;}
		inline double GetXDotDot() const {return p;}
		inline double GetXDotDotDot() const {return j;}
		
		inline void Compute(double& t)
		{
		    tau=(t-t0)/D;
		    
		    if (tau>1.0)
		    {
			    x=xf;v=0.0;p=0.0;j=0.0;
			    return;
		    }
		    x=	  a0 + 
			    a1*tau +
			    a2*pow(tau,2)+
			    a3*pow(tau,3)+
			    a4*pow(tau,4)+
			    a5*pow(tau,5);
		    v=	  a1/D +
			    2*a2*tau/D+
			    3*a3*pow(tau,2)/D+
			    4*a4*pow(tau,3)/D+
			    5*a5*pow(tau,4)/D;
			    
		    p=	  2*a2/pow(D,2)+
			    6*a3*tau/pow(D,2)+
			    12*a4*pow(tau,2)/pow(D,2)+
			    20*a5*pow(tau,3)/pow(D,2);
		    
		    j=	  6*a3/pow(D,3)+
			    24*a4*tau/pow(D,3)+
			    60*a5*pow(tau,2)/pow(D,3);
		}
		
		inline void Create(double mxi,double mvi,double mpi,double mxf,double mt0,double mtf)
		{
		    t0=mt0;
		    tf=mtf;
		    xf=mxf;
		    D=mtf-mt0;
		    a0=mxi;
		    a1=D*mvi;
		    a2=pow(D,2)*mpi/2.0;
		    a3=-1.5*mpi*pow(D,2) - 6*mvi*D + 10*(mxf-mxi);
		    a4= 1.5*mpi*pow(D,2) + 8*mvi*D - 15*(mxf-mxi);
		    a5=-0.5*mpi*pow(D,2) - 3*mvi*D +  6*(mxf-mxi);
		}
		
	private:
		double t0,tf,xf,D,a0,a1,a2,a3,a4,a5;
		double x,v,p,j;
		double tau;
};  
  
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

template<typename value_t>
void ReadTxtFile(const char* filename,std::vector<std::vector<value_t> >& values ) {
    std::string line;
    values.clear();
    std::ifstream myfile (filename);
    std::istringstream iss;
    std::size_t i=0;
    std::size_t nb_vals=0;
    if (myfile.is_open())
    {
        while (getline(myfile,line)) {
            values.push_back(std::vector<value_t>());;
            std::vector<value_t>& v = values[i];
            iss.clear();
            iss.str(line);
            std::copy(std::istream_iterator<value_t>(iss),std::istream_iterator<value_t>(), std::back_inserter(v));
            nb_vals+=v.size();
            i++;
        }
    std::cout << "File ["<<filename<<"] read with success  ["<<nb_vals<<" values, "<<i<<" lines] "<<std::endl;
    }
    else{
     std::cout << "Unable to open file : ["<<filename<<"]"<<std::endl;
    }
    myfile.close();
}

template<typename value_t>
void WriteTxtFile(const char* filename, std::vector<value_t>& values) {
    std::ofstream myfile (filename);
    std::size_t row = 0;
    std::size_t nb_rows = values.size();
    if (myfile.is_open())
    {
        while(row < nb_rows) {
        myfile << values[row] << "\n";
            row++;
        }
    std::cout << "File ["<<filename<<"] write with success  ["<<nb_rows<<" rows ] "<<std::endl;
    }
    else{
     std::cout << "Unable to open file : ["<<filename<<"]"<<std::endl;
    }
    myfile.close();
}

#ifdef INCLUDE_ROS_CODE
class RosNode
{
	public:
		RosNode(std::string ros_node_name)
		{
		  Init(ros_node_name);
		}
		
		RosNode()
		{
		  init_ = false;
		}
		
		void Init(std::string ros_node_name)
		{
		  int argc = 1;
		  char* arg0 = strdup(ros_node_name.c_str());
		  char* argv[] = {arg0, 0};
		  ros::init(argc, argv, ros_node_name,ros::init_options::NoSigintHandler);
		  free (arg0);
		  if(ros::master::check()){
		      ros_nh_ptr_ = new ros::NodeHandle(ros_node_name);
		  }
		  else
		  {
		      std::string err("roscore not found... Did you start the server?");
		      throw std::runtime_error(err);
		  }
		  init_ = true;
		}
		
		~RosNode(){if(ros_nh_ptr_!=NULL && init_ == true){ros_nh_ptr_->shutdown(); delete ros_nh_ptr_;}}
		
		const ros::NodeHandle& GetNode()
		{
		  if(init_ == true) 
		    return *ros_nh_ptr_;
		  else 
		  {
		    std::string err("RosNode not initialized");
		    throw std::runtime_error(err);
		  }
		}
		
	protected:
		ros::NodeHandle* ros_nh_ptr_;
		bool init_;

};

/*template <typename _T >
void operator >>(const YAML::Node& input, _T& value) {
	value = input.as<_T>();
}
template <typename _T >
void operator >> (const YAML::Node &node, std::vector<_T> & v)
{
	for(unsigned i = 0; i < node.size(); i++){
		v.push_back(node[i].as<_T>());
	}
}*/

#ifdef USE_ROS_RT_PUBLISHER

class RealTimePublisherJoints
{
	public:
		
		/** Initialize the real time publisher. */
		RealTimePublisherJoints(const ros::NodeHandle& ros_nh, const std::string topic_name, int msg_size, Eigen::VectorXd init_cond)
		{
			// Checks
			assert(msg_size > 0);
			assert(topic_name.size() > 0);
			
			topic_name_ = topic_name;
			
			assert(init_cond.size() >=  msg_size);
			msg_size_ = msg_size;
			
			pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));
			for(int i = 0; i < msg_size_; i++){
				pub_ptr_->msg_.name.push_back("joint_"+std::to_string(i));
				pub_ptr_->msg_.position.push_back(init_cond[i]);
				
				//pub_ptr_->msg_.position.push_back(0.0);
				//pub_ptr->msg_.velocity.push_back(0.0);
				//pub_ptr->msg_.effort.push_back(0.0);
			}
		}
		/** Publish the topic. */
		inline void publish(const Eigen::Ref<const Eigen::VectorXd>& in)
		{
			if(pub_ptr_ && pub_ptr_->trylock())
			{ 
				pub_ptr_->msg_.header.stamp = ros::Time::now();
				for(int i = 0; i < msg_size_; i++)
				{
					pub_ptr_->msg_.position[i] = in[i];
				}
				pub_ptr_->unlockAndPublish();
			}
		}
		
		inline std::string getTopic(){return topic_name_;}
		
	private:
		
		typedef realtime_tools::RealtimePublisher<sensor_msgs::JointState> rt_publisher_t;
		std::string topic_name_;
		int msg_size_;
		boost::shared_ptr<rt_publisher_t > pub_ptr_;
};

class RealTimePublisherPoseStamped
{
	public:
		
		/** Initialize the real time publisher. */
		RealTimePublisherPoseStamped(const ros::NodeHandle& ros_nh, const std::string topic_name, int msg_size, Eigen::VectorXd init_cond)
		{
			// Checks
			assert(msg_size > 0);
			assert(topic_name.size() > 0);
			
			topic_name_ = topic_name;
			
			assert(init_cond.size() >= 3);

			pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));
			pub_ptr_->msg_.header.frame_id = ref_frame_name; // FIXME
			
			pub_ptr_->msg_.pose.position.x = init_cond[0];
			pub_ptr_->msg_.pose.position.y = init_cond[1];
			pub_ptr_->msg_.pose.position.z = init_cond[2];

			if(init_cond.size() > 3)
			{  
			  pub_ptr_->msg_.pose.orientation.x = init_cond[3];
			  pub_ptr_->msg_.pose.orientation.y = init_cond[4];
			  pub_ptr_->msg_.pose.orientation.z = init_cond[5];
			  pub_ptr_->msg_.pose.orientation.w = init_cond[6];
			}
			else // NOTE no orientation
			{
			  pub_ptr_->msg_.pose.orientation.x = 0.0;
			  pub_ptr_->msg_.pose.orientation.y = 0.0;
			  pub_ptr_->msg_.pose.orientation.z = 0.0;
			  pub_ptr_->msg_.pose.orientation.w = 1.0;
			}
		}
		/** Publish the topic. */
		inline void publish(const Eigen::Ref<const Eigen::VectorXd>& in)
		{
			if(pub_ptr_ && pub_ptr_->trylock())
			{ 
				pub_ptr_->msg_.header.stamp = ros::Time::now();
				
				pub_ptr_->msg_.pose.position.x = in[0];
				pub_ptr_->msg_.pose.position.y = in[1];
				pub_ptr_->msg_.pose.position.z = in[2];
				
				if(in.size() > 3)
				{  
				  pub_ptr_->msg_.pose.orientation.x = in[3];
				  pub_ptr_->msg_.pose.orientation.y = in[4];
				  pub_ptr_->msg_.pose.orientation.z = in[5];
				  pub_ptr_->msg_.pose.orientation.w = in[6];
				
				}
				else // NOTE no orientation
				{
				  pub_ptr_->msg_.pose.orientation.x = 0.0;
				  pub_ptr_->msg_.pose.orientation.y = 0.0;
				  pub_ptr_->msg_.pose.orientation.z = 0.0;
				  pub_ptr_->msg_.pose.orientation.w = 1.0;
				}
				
				
				pub_ptr_->unlockAndPublish();
			}
		}
		
		inline std::string getTopic(){return topic_name_;}
		
	private:
		
		typedef realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> rt_publisher_t;
		std::string topic_name_;
		boost::shared_ptr<rt_publisher_t > pub_ptr_;
};

class RealTimePublisherPath
{
	public:
		
		/** Initialize the real time publisher. */
		RealTimePublisherPath(const ros::NodeHandle& ros_nh, const std::string topic_name, int msg_size, Eigen::VectorXd init_cond)
		{
			// Checks
			assert(msg_size > 0);
			assert(topic_name.size() > 0);
			
			topic_name_ = topic_name;
			
            assert(init_cond.size() >= 2);
			prev_pose_.pose.position.x = init_cond[0];
			prev_pose_.pose.position.y = init_cond[1];
            if(init_cond.size() > 2)
                prev_pose_.pose.position.z = init_cond[2];
            else
                prev_pose_.pose.position.z = 0.0;
			
			//msg_size_ = 2; // NOTE msg_size_ is usless...
			//geometry_msgs::PoseStamped empty_pose;

			pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));
			pub_ptr_->msg_.header.frame_id = ref_frame_name; // FIXME
			for(int i = 0; i < 2; i++){ // NOTE For a line we need two points
				pub_ptr_->msg_.poses.push_back(prev_pose_);
			}
		}
		/** Publish the topic. */
		inline void publish(const Eigen::Ref<const Eigen::VectorXd>& in)
		{
			if(pub_ptr_ && pub_ptr_->trylock())
			{ 
				pub_ptr_->msg_.header.stamp = ros::Time::now();
				//for(int i = 0; i < msg_size_; i++)
				//{
				pub_ptr_->msg_.poses[0] = prev_pose_;
				
				pub_ptr_->msg_.poses[1].pose.position.x = in[0];
				pub_ptr_->msg_.poses[1].pose.position.y = in[1];
                if(in.size() > 2)
                    pub_ptr_->msg_.poses[1].pose.position.z = in[2];
                else
                    pub_ptr_->msg_.poses[1].pose.position.z = 0.0;
				
				prev_pose_ = pub_ptr_->msg_.poses[1];
				
				//}
				pub_ptr_->unlockAndPublish();
			}
		}
		
		inline std::string getTopic(){return topic_name_;}
		
	private:
		
		typedef realtime_tools::RealtimePublisher<nav_msgs::Path> rt_publisher_t;
		std::string topic_name_;
		//int msg_size_;
		geometry_msgs::PoseStamped prev_pose_;
		boost::shared_ptr<rt_publisher_t > pub_ptr_;
};

class RealTimePublisherWrench
{
	public:
		
		/** Initialize the real time publisher. */
        RealTimePublisherWrench(const ros::NodeHandle& ros_nh, const std::string topic_name, int msg_size, Eigen::VectorXd init_cond)
		{
			// Checks
            assert(msg_size > 0);
			assert(topic_name.size() > 0);

			topic_name_ = topic_name;
			
            assert(init_cond.size() >= 2);

            pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));

            pub_ptr_->msg_.wrench.force.x = init_cond[0];
            pub_ptr_->msg_.wrench.force.y = init_cond[1];

            if(init_cond.size() > 2)
                pub_ptr_->msg_.wrench.force.z = init_cond[2];
            else
                pub_ptr_->msg_.wrench.force.z = 0.0;

            if(init_cond.size() > 3)
            {
                pub_ptr_->msg_.wrench.torque.x = init_cond[3];
                pub_ptr_->msg_.wrench.torque.y = init_cond[4];
                pub_ptr_->msg_.wrench.torque.z = init_cond[5];
            }
            pub_ptr_->msg_.header.frame_id = ref_frame_name; // FIXME
		}
		/** Publish the topic. */
		inline void publish(const Eigen::Ref<const Eigen::VectorXd>& in)
		{
			if(pub_ptr_ && pub_ptr_->trylock())
			{ 
				pub_ptr_->msg_.header.stamp = ros::Time::now();

				// Publish the force
				pub_ptr_->msg_.wrench.force.x = in[0];
				pub_ptr_->msg_.wrench.force.y = in[1];

                if(in.size() > 2)
                    pub_ptr_->msg_.wrench.force.z = in[2];
                else
                    pub_ptr_->msg_.wrench.force.z = 0.0;

				// Publish the torques
				if(in.size() > 3) 
				{
				  pub_ptr_->msg_.wrench.torque.x = in[3];
				  pub_ptr_->msg_.wrench.torque.y = in[4];
				  pub_ptr_->msg_.wrench.torque.z = in[5];
				}
				
				pub_ptr_->unlockAndPublish();
			}
		}
		
		inline std::string getTopic(){return topic_name_;}
		
	private:
		
		typedef realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> rt_publisher_t;
		std::string topic_name_;
		boost::shared_ptr<rt_publisher_t > pub_ptr_;

};

class RealTimePublisherSphere
{
	public:
		
		/** Initialize the real time publisher. */
        RealTimePublisherSphere(const ros::NodeHandle& ros_nh, const std::string topic_name, std::string frame_id)
		{
			// Checks
			//assert(msg_size > 0);
			assert(topic_name.size() > 0);
			
			topic_name_ = topic_name;
			
			//assert(init_cond.size() >= 3);
			pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));
			
			pub_ptr_->msg_.header.frame_id = frame_id;
			
			pub_ptr_->msg_.type = visualization_msgs::Marker::SPHERE; // FIXME Hardcored sphere
			pub_ptr_->msg_.action = visualization_msgs::Marker::ADD;
			pub_ptr_->msg_.ns = "marker";
			pub_ptr_->msg_.id = 0;
			
			pub_ptr_->msg_.color.r = 0.0;
			pub_ptr_->msg_.color.g = 0.0;
			pub_ptr_->msg_.color.b = 1.0;
	  
			//marker.pose.orientation.x = 0.0; // FIXME I can use these infos to add the covariance (rotation)
			//marker.pose.orientation.y = 0.0;
			//marker.pose.orientation.z = 0.0;
			pub_ptr_->msg_.pose.orientation.w = 1.0;
            pub_ptr_->msg_.color.a = 0.5;
			  
		}
		/** Publish the topic. */
		inline void publish(const Eigen::Ref<const Eigen::VectorXd>& in)
		{
			if(pub_ptr_ && pub_ptr_->trylock())
			{ 
				pub_ptr_->msg_.header.stamp = ros::Time::now();

				// Mean
				pub_ptr_->msg_.pose.position.x = in[0];
				pub_ptr_->msg_.pose.position.y = in[1];
				pub_ptr_->msg_.pose.position.z = in[2];
				
				// Variance
				pub_ptr_->msg_.scale.x = 0.01 + 200 * in[3]; //* in[3];
				pub_ptr_->msg_.scale.y = 0.01 + 200 * in[4]; // in[4];
				pub_ptr_->msg_.scale.z = 0.01 + 200 * in[5]; //* in[5];

				pub_ptr_->unlockAndPublish();
			}
		}
		
		inline std::string getTopic(){return topic_name_;}
		
	private:
		
		typedef realtime_tools::RealtimePublisher<visualization_msgs::Marker> rt_publisher_t;
		std::string topic_name_;
		boost::shared_ptr<rt_publisher_t > pub_ptr_;
		//visualization_msgs::Marker marker_;
};

//typedef std::map<std::string,std::pair<Eigen::VectorXd*,RealTimePublisher_t*> > pubs_map_t;
//typedef pubs_map_t::iterator pubs_map_it_t;

template <class RealTimePublisher_t>
class RealTimePublishers
{
	public:

		RealTimePublishers(){};
// 		~RealTimePublishers()
// 		{
// 			for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
// 				delete iterator->second.second;
// 		}
		
		// Add a RealTimePublisher already created
        void AddPublisher(boost::shared_ptr<RealTimePublisher_t> pub_ptr, Eigen::VectorXd* vector_ptr)
		{
			assert(pub_ptr!=false);
			// Put it into the map with his friend
			map_[pub_ptr->getTopic()] = std::make_pair(vector_ptr,pub_ptr);
        }
		
        // For publishers with FRAME
        /*void AddPublisher(const ros::NodeHandle& ros_nh, const std::string topic_name, Eigen::VectorXd* vector_ptr, const std::string frame)
        {
            boost::shared_ptr<RealTimePublisher_t> pub_ptr = boost::make_shared<RealTimePublisher_t>(ros_nh,topic_name,frame);
            assert(pub_ptr!=false);
            // Put it into the map with his friend
            map_[pub_ptr->getTopic()] = std::make_pair(vector_ptr,pub_ptr);
        }*/

		// Add a new fresh RealTimePublisher
		void AddPublisher(const ros::NodeHandle& ros_nh, const std::string topic_name, int msg_size, Eigen::VectorXd* vector_ptr, Eigen::VectorXd init_cond = Eigen::VectorXd::Zero(50)) //HACK 50 because it's rare to have more then 50 dofs...
		{
			boost::shared_ptr<RealTimePublisher_t> pub_ptr = boost::make_shared<RealTimePublisher_t>(ros_nh,topic_name,msg_size,init_cond);
			assert(pub_ptr!=false);
			// Put it into the map with his friend
			map_[pub_ptr->getTopic()] = std::make_pair(vector_ptr,pub_ptr);
		}
		void PublishAll()
		{
			for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
				iterator->second.second->publish(*iterator->second.first);
		}
	private:
		
		//typedef std::map<std::string,std::pair<Eigen::VectorXd*,RealTimePublisher_t*> > pubs_map_t;
		typedef std::map<std::string,std::pair<Eigen::VectorXd*,boost::shared_ptr<RealTimePublisher_t> > > pubs_map_t;
		typedef typename pubs_map_t::iterator pubs_map_it_t;
		
		pubs_map_t map_;
};


#endif // RT PUBLISHERS
#endif // ROS

}

#endif // TOOLBOX_H


