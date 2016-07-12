#ifndef TOOLBOX_H
#define TOOLBOX_H

////////// STD
#include <iostream>
#include <fstream>
#include <iterator>
#include <cmath>

#ifdef INCLUDE_ROS_CODE
	////////// ROS
	#include <ros/ros.h>
	#ifdef USE_ROS_RT_PUBLISHER
	//#ifdef INCLUDE_ROS_RT_PUBLISHER
          //#define USE_ROS_RT_PUBLISHER
      #include <std_msgs/Float64MultiArray.h>
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
#include <boost/thread.hpp>

namespace tool_box 
{

inline void Delete(const int idx, Eigen::VectorXd& vect)
{
    int n = vect.size()-idx-1;
    vect.segment(idx,n) = vect.tail(n);
    vect.conservativeResize(vect.size()-1);
}

inline void PushBack(const double value, Eigen::VectorXd& vect)
{
    int n = vect.size();
    vect.conservativeResize(n+1,Eigen::NoChange);
    vect(n) = value;
}

inline void PushBack(const Eigen::ArrayXd vect, Eigen::MatrixXd& mat)
{
    assert(vect.size() == mat.cols());
    int n = mat.rows();
    mat.conservativeResize(n+1,Eigen::NoChange);
    mat.row(n) = vect;
}

inline bool CropData(Eigen::MatrixXd& data, const double dt = 0.1, const double dist_min = 0.01)
{
    Eigen::MatrixXd data_tmp = data;
    data.resize(0,data_tmp.cols());

    for(int i = 0; i < data_tmp.rows()-1; i++)
        if((data_tmp.row(i+1) - data_tmp.row(i)).norm() > dt*dist_min)
        {
            PushBack(data_tmp.row(i),data);
        }

    if(data.rows() == 0)
    {
        std::cerr << "Data is empty, did you move the robot?" << std::endl;
        return false;
    }
    else
        return true;
}

class AsyncThread
{
    typedef boost::function<void ()> funct_t;
    public:
        AsyncThread()
        {
            trigger_ = false;
            stop_loop_ = false;
            loop_ = boost::thread(boost::bind(&AsyncThread::Loop, this));
        }

        ~AsyncThread()
        {
            //std::cout << "Destroy" << std::endl;
            stop_loop_ = true;
            callback_.join();
            loop_.join();
        }

        inline void AddHandler(funct_t f)
        {
            f_ = f;
        }

        inline void Trigger()
        {
            //std::cout << "Set the Trigger" << std::endl;
            trigger_ = true;
        }

        inline void Loop()
        {
            while(!stop_loop_)
            {
                if(trigger_)
                {
                    if(!f_.empty())
                    {
                        //std::cout << "Launching the Callback" << std::endl;
                        trigger_ = false;
                        callback_ =  boost::thread(f_);
                        callback_.join();
                    }
                    else
                        std::cerr << "No Callback function" << std::endl;
                }
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            }
            return;
        }

    private:
        funct_t f_;
        boost::atomic<bool> trigger_;
        boost::atomic<bool> stop_loop_;
        boost::thread loop_;
        boost::thread callback_;
};

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
            x=	a0 +
			    a1*tau +
			    a2*pow(tau,2)+
			    a3*pow(tau,3)+
			    a4*pow(tau,4)+
			    a5*pow(tau,5);
            v=	a1/D +
			    2*a2*tau/D+
			    3*a3*pow(tau,2)/D+
			    4*a4*pow(tau,3)/D+
			    5*a5*pow(tau,4)/D;
			    
            p=	2*a2/pow(D,2)+
			    6*a3*tau/pow(D,2)+
			    12*a4*pow(tau,2)/pow(D,2)+
			    20*a5*pow(tau,3)/pow(D,2);
		    
            j=	6*a3/pow(D,3)+
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

template<typename value_t>
void WriteTxtFile(const char* filename, std::vector<std::vector<value_t> >& values ) {
    std::ofstream myfile (filename);
    std::size_t row = 0;
    std::size_t nb_rows = values.size();
    std::size_t col = 0;
    std::size_t nb_cols = values[0].size();

    if (myfile.is_open())
    {
        while(row < nb_rows) {
            while(col < nb_cols) {
                myfile << values[row][col] << " ";
                col++;
            }
            col = 0;
            row++;
            myfile << "\n";
        }
    std::cout << "File ["<<filename<<"] write with success  ["<<nb_rows<<" rows ]["<<nb_cols<<" cols ] "<<std::endl;
    }
    else{
     std::cout << "Unable to open file : ["<<filename<<"]"<<std::endl;
    }
    myfile.close();
}

inline double Intersection2Width(double center1, double center2, double ratio)
{
    assert(ratio > 0.0 && ratio <= 1);
    return std::abs(center2-center1)/(2.0*std::sqrt(2.0*std::log(1.0/ratio)));
}

inline double GaussMf(double x, double c, double sigma)
{
    assert(sigma>0);
    return std::exp(-(x-c)*(x-c)/(2.0*sigma*sigma));
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

class RealTimePublisherVector
{
	public:
		
		/** Initialize the real time publisher. */
        RealTimePublisherVector(const ros::NodeHandle& ros_nh, const std::string topic_name)
		{
			// Checks
			assert(topic_name.size() > 0);
			topic_name_ = topic_name;
			pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));

		}
		/** Publish the topic. */
        inline void Publish(const Eigen::Ref<const Eigen::VectorXd>& in)
		{
			if(pub_ptr_ && pub_ptr_->trylock())
			{ 
                //pub_ptr_->msg_.header.stamp = ros::Time::now();
                int data_size = pub_ptr_->msg_.data.size();
                //assert(data_size >= in.size());
                for(int i = 0; i < data_size; i++)
                {
                    pub_ptr_->msg_.data[i] = in(i);
                }
				pub_ptr_->unlockAndPublish();
			}
		}

        /** Remove an element in the vector. */
        inline void Remove(const int idx)
        {
            if(pub_ptr_)
            {
                pub_ptr_->lock();
                pub_ptr_->msg_.data.erase(pub_ptr_->msg_.data.begin()+idx);
                pub_ptr_->unlock();
            }
        }

        /** Resize the vector. */
        inline void Resize(const int dim)
        {
            if(pub_ptr_)
            {
                pub_ptr_->lock();
                pub_ptr_->msg_.data.resize(dim);
                pub_ptr_->unlock();
            }
        }

        /** Add a new element at the back. */
        inline void PushBackEmpty()
        {
            if(pub_ptr_)
            {
                pub_ptr_->lock();
                pub_ptr_->msg_.data.push_back(0.0);
                pub_ptr_->unlock();
            }
        }
		
		inline std::string getTopic(){return topic_name_;}
		
	private:	
        typedef realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> rt_publisher_t;
		std::string topic_name_;
		int msg_size_;
		boost::shared_ptr<rt_publisher_t > pub_ptr_;
};

template <class RealTimePublisher_t>
class RealTimePublishers
{
	public:

		RealTimePublishers(){};

		// Add a RealTimePublisher already created
        void AddPublisher(boost::shared_ptr<RealTimePublisher_t> pub_ptr, Eigen::VectorXd* vector_ptr)
		{
			assert(pub_ptr!=false);
			// Put it into the map with his friend
			map_[pub_ptr->getTopic()] = std::make_pair(vector_ptr,pub_ptr);
        }

		// Add a new fresh RealTimePublisher
        void AddPublisher(const ros::NodeHandle& ros_nh, const std::string topic_name, Eigen::VectorXd* vector_ptr)
		{
            boost::shared_ptr<RealTimePublisher_t> pub_ptr = boost::make_shared<RealTimePublisher_t>(ros_nh,topic_name);
			assert(pub_ptr!=false);
			// Put it into the map with his friend
			map_[pub_ptr->getTopic()] = std::make_pair(vector_ptr,pub_ptr);
		}

        // Remove an element from all the publishers
        void RemoveAll(const int idx)
        {
            for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
                iterator->second.second->Remove(idx);
        }

        // Resize all the publishers
        void ResizeAll(const int dim)
        {
            for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
                iterator->second.second->Resize(dim);
        }

        // Push back an empty value in all the publishers
        void PushBackEmptyAll()
        {
            for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
                iterator->second.second->PushBackEmpty();
        }

        // Publish!
		void PublishAll()
		{
			for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
                iterator->second.second->Publish(*iterator->second.first);
		}
	private:
		
		typedef std::map<std::string,std::pair<Eigen::VectorXd*,boost::shared_ptr<RealTimePublisher_t> > > pubs_map_t;
		typedef typename pubs_map_t::iterator pubs_map_it_t;
		
		pubs_map_t map_;
};


#endif // RT PUBLISHERS
#endif // ROS

}

#endif // TOOLBOX_H


