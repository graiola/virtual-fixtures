#ifndef TOOLBOX_H
#define TOOLBOX_H

////////// STD
#include <iostream>
#include <fstream>

////////// ROS
#include <ros/ros.h>

////////// Eigen
#include <eigen3/Eigen/Core>

////////// BOOST
#include <boost/bind.hpp>

namespace tool_box 
{

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
		    /*if (tmp_cnt++%100==0)
			    M3_INFO("T: %f Tau: %f ",t,tau);
		    if (tmp_cnt%100==0)	
			    M3_INFO("X %f\n",x);*/
		    
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
		    /*M3_INFO("Create t0 %f\n",t0);
		    M3_INFO("Create tf %f\n",tf);
		    M3_INFO("Create xf %f\n",mxf);
		    M3_INFO("Create xi %f\n",mxi);
		    M3_INFO("Create vi %f\n",mvi);
		    M3_INFO("Create pi %f\n",mpi);
		    M3_INFO("Create a1 %f\n",a1);
		    M3_INFO("Create a2 %f\n",a2);
		    M3_INFO("Create a3 %f\n",a3);
		    M3_INFO("Create a4 %f\n",a4);
		    M3_INFO("Create a5 %f\n",a5);*/
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
  
	
}

#endif

