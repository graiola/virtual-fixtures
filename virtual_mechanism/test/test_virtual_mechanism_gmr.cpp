#include <gtest/gtest.h>
#include "virtual_mechanism/virtual_mechanism_gmr.h"

////////// STD
#include <iostream>
#include <fstream> 
#include <iterator>
#include <boost/concept_check.hpp>

////////// Function Approximator
#include <functionapproximators/FunctionApproximatorGMR.hpp>
#include <functionapproximators/MetaParametersGMR.hpp>
#include <functionapproximators/ModelParametersGMR.hpp>

using namespace virtual_mechanism_interface;
using namespace virtual_mechanism_gmr;
using namespace Eigen;
using namespace boost;
using namespace DmpBbo;

int test_dim = 3;
double dt = 0.001;

fa_t* generateDemoFa(){
	
  std::string file_name = "/home/gennaro/catkin_ws/src/virtual-fixtures/virtual_mechanism/test/gmm_1.txt"; // FIXME
  
  ModelParametersGMR* model_parameters_gmr = ModelParametersGMR::loadGMMFromMatrix(file_name);
  FunctionApproximatorGMR* fa_ptr = new FunctionApproximatorGMR(model_parameters_gmr);
  
  return fa_ptr;  
}

TEST(VirtualMechanismGmrTest, InitializesCorrectly)
{
  
  ::testing::FLAGS_gtest_death_test_style = "threadsafe"; // NOTE https://code.google.com/p/googletest/wiki/AdvancedGuide#Death_Test_Styles
  
  boost::shared_ptr<fa_t> fa_ptr(generateDemoFa());
  
  //ASSERT_DEATH(VirtualMechanismGmr(1,fa_ptr),".*");
  //ASSERT_DEATH(VirtualMechanismGmr(2,fa_ptr),".*");
  EXPECT_NO_THROW(VirtualMechanismGmr<VirtualMechanismInterfaceFirstOrder>(3,fa_ptr));
  EXPECT_NO_THROW(VirtualMechanismGmr<VirtualMechanismInterfaceSecondOrder>(3,fa_ptr));
}

TEST(VirtualMechanismGmrTest, UpdateMethod)
{
  boost::shared_ptr<fa_t> fa_ptr(generateDemoFa());
  
  VirtualMechanismGmr<VirtualMechanismInterfaceFirstOrder> vm1(test_dim,fa_ptr);
  VirtualMechanismGmr<VirtualMechanismInterfaceSecondOrder> vm2(test_dim,fa_ptr);
  
  // Force input interface
  Eigen::VectorXd force;
  force.resize(test_dim);
  force.fill(1.0);
  //force << 100.0, 0.0 , 0.0;
  
  EXPECT_NO_THROW(vm1.Update(force,dt));
  EXPECT_NO_THROW(vm2.Update(force,dt));
  
  
//   Eigen::VectorXd state;
//   state.resize(test_dim);
//   vm.getState(state);
//   std::cout<<state<<std::endl;
//   getchar();
  
  // Cart input interface
  Eigen::VectorXd pos;
  pos.resize(test_dim);
  Eigen::VectorXd vel;
  vel.resize(test_dim);
  EXPECT_NO_THROW(vm1.Update(pos,vel,dt));
  EXPECT_NO_THROW(vm2.Update(pos,vel,dt));
}

TEST(VirtualMechanismGmrTest, LoopUpdateMethod)
{
  boost::shared_ptr<fa_t> fa_ptr(generateDemoFa());
  
  double phase, phase_dot;
  
  VirtualMechanismGmr<VirtualMechanismInterfaceFirstOrder> vm1(test_dim,fa_ptr);
  VirtualMechanismGmr<VirtualMechanismInterfaceSecondOrder> vm2(test_dim,fa_ptr);
  
  // Force input interface
  Eigen::VectorXd force;
  force.resize(test_dim);
  force.fill(100.0);
  
  // State
  Eigen::VectorXd state;
  state.resize(test_dim);
  Eigen::VectorXd state_dot;
  state_dot.resize(test_dim);
  
  for (int i = 0; i < 500; i++)
  {
    vm1.Update(force,dt);
    vm1.getState(state);
    vm1.getStateDot(state_dot);
    phase = vm1.getPhase();
    phase_dot = vm1.getPhaseDot();
    
    //std::cout<<"*******"<<std::endl;
    //std::cout<<"STATE\n "<<state<<std::endl;
    //std::cout<<"PHASE "<<phase<<std::endl;
    //std::cout<<"PHASE_DOT "<<phase_dot<<std::endl;
  }
  EXPECT_NO_THROW(vm1.getStateDot(state_dot));
  
  for (int i = 0; i < 500; i++)
  {
    vm2.Update(force,dt);
    vm2.getState(state);
    vm2.getStateDot(state_dot);
    phase = vm2.getPhase();
    phase_dot = vm2.getPhaseDot();
    
    //std::cout<<"*******"<<std::endl;
    //std::cout<<"STATE\n "<<state<<std::endl;
    //std::cout<<"PHASE "<<phase<<std::endl;
    //std::cout<<"PHASE_DOT "<<phase_dot<<std::endl;
  }
  EXPECT_NO_THROW(vm2.getStateDot(state_dot));
  
}

TEST(VirtualMechanismGmrTest, GetMethods)
{
  boost::shared_ptr<fa_t> fa_ptr(generateDemoFa());
  
  VirtualMechanismGmr<VirtualMechanismInterfaceFirstOrder> vm1(test_dim,fa_ptr);
  VirtualMechanismGmr<VirtualMechanismInterfaceSecondOrder> vm2(test_dim,fa_ptr);
  
  // State
  Eigen::VectorXd state;
  state.resize(test_dim);
  EXPECT_NO_THROW(vm1.getState(state));
  EXPECT_NO_THROW(vm2.getState(state));
  
  // StateDot
  Eigen::VectorXd state_dot;
  state_dot.resize(test_dim);
  EXPECT_NO_THROW(vm1.getStateDot(state_dot));
  EXPECT_NO_THROW(vm2.getStateDot(state_dot));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}