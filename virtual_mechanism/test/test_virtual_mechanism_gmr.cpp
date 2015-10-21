#include <toolbox/debug.h>

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

double dt = 0.001;

int position_dim = 3;
int state_dim = 7;
int expected_gmm_dim = 7;

typedef VirtualMechanismInterfaceFirstOrder VMP_1ord_t;
typedef VirtualMechanismInterfaceSecondOrder VMP_2ord_t;

fa_t* generateDemoFa(){
	
  std::string file_name = "/home/gennaro/catkin_ws/src/virtual-fixtures/virtual_mechanism/test/wonderfall_1.txt"; // FIXME
  
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
  EXPECT_NO_THROW(VirtualMechanismGmr<VMP_1ord_t>(expected_gmm_dim,fa_ptr));
  EXPECT_NO_THROW(VirtualMechanismGmr<VMP_2ord_t>(expected_gmm_dim,fa_ptr));
}

TEST(VirtualMechanismGmrTest, UpdateMethod)
{
  boost::shared_ptr<fa_t> fa_ptr(generateDemoFa());
  
  VirtualMechanismGmr<VMP_1ord_t> vm1(expected_gmm_dim,fa_ptr);
  VirtualMechanismGmr<VMP_2ord_t> vm2(expected_gmm_dim,fa_ptr);
  
  Eigen::VectorXd force(position_dim);
  Eigen::VectorXd pos(position_dim);
  Eigen::VectorXd vel(position_dim);
  force.fill(1.0);
  
  START_REAL_TIME_CRITICAL_CODE();
  
  // Force input interface
  EXPECT_NO_THROW(vm1.Update(force,dt));
  EXPECT_NO_THROW(vm2.Update(force,dt));
  
  // Cart input interface
  EXPECT_NO_THROW(vm1.Update(pos,vel,dt));
  EXPECT_NO_THROW(vm2.Update(pos,vel,dt));
  
  END_REAL_TIME_CRITICAL_CODE();
  
}

TEST(VirtualMechanismGmrTest, GetProbability)
{
  boost::shared_ptr<fa_t> fa_ptr(generateDemoFa());
  
  VirtualMechanismGmr<VMP_1ord_t> vm1(expected_gmm_dim,fa_ptr);
  VirtualMechanismGmr<VMP_2ord_t> vm2(expected_gmm_dim,fa_ptr);
  
  Eigen::VectorXd pos(position_dim);
  pos.fill(1.0);
  
  START_REAL_TIME_CRITICAL_CODE();
  
  EXPECT_NO_THROW(vm1.getProbability(pos));
  EXPECT_NO_THROW(vm2.getProbability(pos));
  
  END_REAL_TIME_CRITICAL_CODE();
}

TEST(VirtualMechanismGmrTest, GetDistance)
{
  boost::shared_ptr<fa_t> fa_ptr(generateDemoFa());
  
  VirtualMechanismGmr<VMP_1ord_t> vm1(expected_gmm_dim,fa_ptr);
  VirtualMechanismGmr<VMP_2ord_t> vm2(expected_gmm_dim,fa_ptr);
  
  Eigen::VectorXd pos(position_dim);
  pos.fill(1.0);
  
  START_REAL_TIME_CRITICAL_CODE();
  
  EXPECT_NO_THROW(vm1.getDistance(pos));
  EXPECT_NO_THROW(vm2.getDistance(pos));
  
  END_REAL_TIME_CRITICAL_CODE();
  
  vm1.setWeightedDist(true);
  vm2.setWeightedDist(true);
  
  START_REAL_TIME_CRITICAL_CODE();
  
  EXPECT_NO_THROW(vm1.getDistance(pos));
  EXPECT_NO_THROW(vm2.getDistance(pos));
  
  END_REAL_TIME_CRITICAL_CODE();
  
}

/*TEST(VirtualMechanismGmrTest, LoopUpdateMethod)
{
  boost::shared_ptr<fa_t> fa_ptr(generateDemoFa());
  
  double phase, phase_dot;
  
  VirtualMechanismGmr<VMP_1ord_t> vm1(test_dim,fa_ptr);
  VirtualMechanismGmr<VMP_2ord_t> vm2(test_dim,fa_ptr);
  
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
}*/

TEST(VirtualMechanismGmrTest, GetMethods)
{
  boost::shared_ptr<fa_t> fa_ptr(generateDemoFa());
  
  VirtualMechanismGmr<VMP_1ord_t> vm1(expected_gmm_dim,fa_ptr);
  VirtualMechanismGmr<VMP_2ord_t> vm2(expected_gmm_dim,fa_ptr);
  
  // State
  Eigen::VectorXd state;
  state.resize(state_dim);
  EXPECT_NO_THROW(vm1.getState(state));
  EXPECT_NO_THROW(vm2.getState(state));
  
  // PositionDot
  Eigen::VectorXd position_dot;
  position_dot.resize(position_dim);
  EXPECT_NO_THROW(vm1.getPositionDot(position_dot));
  EXPECT_NO_THROW(vm2.getPositionDot(position_dot));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}