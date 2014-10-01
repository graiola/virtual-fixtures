#include <gtest/gtest.h>
#include "virtual_mechanism/virtual_mechanism_dmp.h"

//#include <dynamicalsystems/DynamicalSystem.hpp>
//#include <dynamicalsystems/ExponentialSystem.hpp>
//#include <dynamicalsystems/SigmoidSystem.hpp>
//#include <dynamicalsystems/TimeSystem.hpp>
//#include <dynamicalsystems/SpringDamperSystem.hpp>

#include <dmp/Trajectory.hpp>
#include <functionapproximators/FunctionApproximatorGMR.hpp>
#include <functionapproximators/MetaParametersGMR.hpp>
#include <functionapproximators/ModelParametersGMR.hpp>

using namespace virtual_mechanism_dmp;
using namespace Eigen;
using namespace boost;
using namespace DmpBbo;

int test_dim = 3;
double dt = 0.001;

Trajectory generateViapointTrajectory(const VectorXd& ts, const VectorXd& y_first, const VectorXd& y_last, const double& Tf, const double& Ti)
{
    //VectorXd y_first = VectorXd::Zero(n_dims);
    //VectorXd y_last  = VectorXd::Ones(n_dims) * 0.3;
    
    assert(y_first.size() == y_last.size());
    
    int n_dims = y_first.size();
    double viapoint_time = (Tf -Ti)/2;

    VectorXd y_yd_ydd_viapoint = VectorXd::Zero(3*n_dims);
    
    for(int i = 0; i<n_dims; i++)
	    y_yd_ydd_viapoint[i] = (y_last[i] - y_first[i])/2;
    
    return  Trajectory::generatePolynomialTrajectoryThroughViapoint(ts,y_first,y_yd_ydd_viapoint,viapoint_time,y_last);
}

dmp_t* generateDemoDmp(const VectorXd y_init, const VectorXd y_attr, const double dt, const int Ndof, const double Ti, const double Tf, int& n_time_steps_trajectory){

	assert(y_init.size() == Ndof);
	assert(y_attr.size() == Ndof);
	
	// GENERATE A TRAJECTORY
	n_time_steps_trajectory = (int)((Tf-Ti)/dt) + 1;
	
	// Some default values for dynamical system
	//VectorXd y_init = VectorXd::Zero(Ndof);
	//VectorXd y_attr  = VectorXd::Ones(Ndof) * 0.4;
	
	VectorXd ts = VectorXd::LinSpaced(n_time_steps_trajectory,Ti,Tf); // From Ti to Tf in n_time_steps_trajectory steps
	
	Trajectory trajectory = generateViapointTrajectory(ts, y_init, y_attr, Tf, Ti);
  
        
	// MAKE THE FUNCTION APPROXIMATORS
	
	int input_dim = 1;
	int n_basis_functions = 25;
	
	
        // LWR
        /*double overlap = 0.01;
	MetaParametersLWR* meta_parameters = new MetaParametersLWR(input_dim,n_basis_functions,overlap);
        FunctionApproximatorLWR* fa = new FunctionApproximatorLWR(meta_parameters);*/
	
	// GMR
	MetaParametersGMR* meta_parameters_gmr = new MetaParametersGMR(input_dim,n_basis_functions);
	FunctionApproximatorGMR* fa = new FunctionApproximatorGMR(meta_parameters_gmr);
	
	//Dmp::DmpType dmp_type = Dmp::KULVICIUS_2012_JOINING;
	dmp_t::DmpType dmp_type = dmp_t::IJSPEERT_2002_MOVEMENT;
	
	std::vector<FunctionApproximator*> function_approximators(Ndof);    	
	for (int dd=0; dd<Ndof; dd++)
		function_approximators[dd] = fa->clone();
	
	//Dmp(double tau, Eigen::VectorXd y_init, Eigen::VectorXd y_attr, std::vector<FunctionApproximator*> function_approximators, DmpType dmp_type=KULVICIUS_2012_JOINING);
	
	dmp_t* dmp = new dmp_t(Tf,y_init,y_attr,function_approximators,dmp_type);
	
	dmp->train(trajectory);
	
	//dmp->train(inputs,targets);
	
	return dmp;  
}

TEST(VirtualMechanismDmpTest, InitializesCorrectly)
{
  
  ::testing::FLAGS_gtest_death_test_style = "threadsafe"; // NOTE https://code.google.com/p/googletest/wiki/AdvancedGuide#Death_Test_Styles
  
  int n_time_steps_trajectory = 0;
  VectorXd y_attr;
  VectorXd y_init = VectorXd::Zero(test_dim);
  
  y_attr.resize(test_dim);
  y_attr << 0.24, -0.1, -0.53; 

  boost::shared_ptr<dmp_t> dmp_ptr(generateDemoDmp(y_init,y_attr,dt,test_dim,0.0,3.0,n_time_steps_trajectory));
  
  ASSERT_DEATH(VirtualMechanismDmp(1,dmp_ptr),".*");
  ASSERT_DEATH(VirtualMechanismDmp(2,dmp_ptr),".*");
  EXPECT_NO_THROW(VirtualMechanismDmp(3,dmp_ptr));
}

TEST(VirtualMechanismDmpTest, UpdateMethod)
{
  
  int n_time_steps_trajectory = 0;
  VectorXd y_attr;
  VectorXd y_init = VectorXd::Zero(test_dim);
  
  y_attr.resize(test_dim);
  y_attr << 0.24, -0.1, -0.53; 

  boost::shared_ptr<dmp_t> dmp_ptr(generateDemoDmp(y_init,y_attr,dt,test_dim,0.0,3.0,n_time_steps_trajectory));
  
  VirtualMechanismDmp vm(test_dim,dmp_ptr);
  
  // Force input interface
  Eigen::VectorXd force;
  force.resize(test_dim);
  EXPECT_NO_THROW(vm.Update(force,dt));
  
  // Cart input interface
  Eigen::VectorXd pos;
  pos.resize(test_dim);
  Eigen::VectorXd vel;
  vel.resize(test_dim);
  EXPECT_NO_THROW(vm.Update(pos,vel,dt));
}

TEST(VirtualMechanismDmpTest, GetMethods)
{
  int n_time_steps_trajectory = 0;
  VectorXd y_attr;
  VectorXd y_init = VectorXd::Zero(test_dim);
  
  y_attr.resize(test_dim);
  y_attr << 0.24, -0.1, -0.53; 

  boost::shared_ptr<dmp_t> dmp_ptr(generateDemoDmp(y_init,y_attr,dt,test_dim,0.0,3.0,n_time_steps_trajectory));
  
  VirtualMechanismDmp vm(test_dim,dmp_ptr);
  
  // State
  Eigen::VectorXd state;
  state.resize(test_dim);
  EXPECT_NO_THROW(vm.getState(state));
  
  // StateDot
  Eigen::VectorXd state_dot;
  state_dot.resize(test_dim);
  EXPECT_NO_THROW(vm.getStateDot(state_dot));
 
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}