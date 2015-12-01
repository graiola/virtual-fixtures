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

int test_dim = 3;
double dt = 0.001;

typedef VirtualMechanismInterfaceFirstOrder VMP_1ord_t;
typedef VirtualMechanismInterfaceSecondOrder VMP_2ord_t;

fa_t* generateDemoFaPos(){
	
  std::string model_name = "/home/meka/catkin_ws/src/virtual-fixtures/mechanism_manager/models/cool_1.txt"; // FIXME
  
  ModelParametersGMR* model_parameters_gmr = ModelParametersGMR::loadGMMFromMatrix(model_name);
  FunctionApproximatorGMR* fa_ptr = new FunctionApproximatorGMR(model_parameters_gmr);
  
  return fa_ptr;  
}

fa_t* generateDemoFaPhaseDot(){
        
  std::string phase_dot_model_name = "/home/meka/catkin_ws/src/virtual-fixtures/mechanism_manager/models/cool_sdot_1.txt"; // FIXME
  
  ModelParametersGMR* model_parameters_gmr = ModelParametersGMR::loadGMMFromMatrix(phase_dot_model_name);
  FunctionApproximatorGMR* fa_ptr = new FunctionApproximatorGMR(model_parameters_gmr);
  
  return fa_ptr;  
}

TEST(VirtualMechanismGmrTest, InitializesCorrectly)
{
  
  ::testing::FLAGS_gtest_death_test_style = "threadsafe"; // NOTE https://code.google.com/p/googletest/wiki/AdvancedGuide#Death_Test_Styles
  
  boost::shared_ptr<fa_t> fa_pos_ptr(generateDemoFaPos());
  boost::shared_ptr<fa_t> fa_phase_dot_ptr(generateDemoFaPhaseDot());
  
  //ASSERT_DEATH(VirtualMechanismGmr(1,fa_ptr),".*");
  //ASSERT_DEATH(VirtualMechanismGmr(2,fa_ptr),".*");
  EXPECT_NO_THROW(VirtualMechanismGmr<VMP_1ord_t>(3,fa_pos_ptr,fa_phase_dot_ptr));
  EXPECT_NO_THROW(VirtualMechanismGmr<VMP_2ord_t>(3,fa_pos_ptr,fa_phase_dot_ptr));
}

TEST(VirtualMechanismGmrTest, UpdateMethod)
{
  boost::shared_ptr<fa_t> fa_pos_ptr(generateDemoFaPos());
  boost::shared_ptr<fa_t> fa_phase_dot_ptr(generateDemoFaPhaseDot());
  
  VirtualMechanismGmr<VMP_1ord_t> vm1(test_dim,fa_pos_ptr,fa_phase_dot_ptr);
  VirtualMechanismGmr<VMP_2ord_t> vm2(test_dim,fa_pos_ptr,fa_phase_dot_ptr);

  
  Eigen::VectorXd force(test_dim);
  Eigen::VectorXd pos(test_dim);
  Eigen::VectorXd vel(test_dim);
   
  
  START_REAL_TIME_CRITICAL_CODE();
  
  // Passive VM
  vm1.setActive(false);
  vm2.setActive(false);
  
  force.fill(1.0);
  
  // Force input interface
  EXPECT_NO_THROW(vm1.Update(force,dt));
  EXPECT_NO_THROW(vm2.Update(force,dt));
  
  // Cart input interface
  EXPECT_NO_THROW(vm1.Update(pos,vel,dt));
  EXPECT_NO_THROW(vm2.Update(pos,vel,dt));
  
  // Active VM
  vm1.setActive(true);
  vm2.setActive(true);
  
  force.fill(0.0); // Doesn't really matter to have it at zero since the system switched dynamics
  
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
  boost::shared_ptr<fa_t> fa_pos_ptr(generateDemoFaPos());
  boost::shared_ptr<fa_t> fa_phase_dot_ptr(generateDemoFaPhaseDot());
  
  VirtualMechanismGmr<VMP_1ord_t> vm1(test_dim,fa_pos_ptr,fa_phase_dot_ptr);
  VirtualMechanismGmr<VMP_2ord_t> vm2(test_dim,fa_pos_ptr,fa_phase_dot_ptr);
  
  Eigen::VectorXd pos(test_dim);
  pos.fill(1.0);
  
  START_REAL_TIME_CRITICAL_CODE();
  
  EXPECT_NO_THROW(vm1.getProbability(pos));
  EXPECT_NO_THROW(vm2.getProbability(pos));
  
  END_REAL_TIME_CRITICAL_CODE();
}

TEST(VirtualMechanismGmrTest, GetDistance)
{
  boost::shared_ptr<fa_t> fa_pos_ptr(generateDemoFaPos());
  boost::shared_ptr<fa_t> fa_phase_dot_ptr(generateDemoFaPhaseDot());
  
  VirtualMechanismGmr<VMP_1ord_t> vm1(test_dim,fa_pos_ptr,fa_phase_dot_ptr);
  VirtualMechanismGmr<VMP_2ord_t> vm2(test_dim,fa_pos_ptr,fa_phase_dot_ptr);
  
  Eigen::VectorXd pos(test_dim);
  pos.fill(1.0);
  
  vm1.setWeightedDist(false);
  vm2.setWeightedDist(false);
  
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
  boost::shared_ptr<fa_t> fa_pos_ptr(generateDemoFaPos());
  boost::shared_ptr<fa_t> fa_phase_dot_ptr(generateDemoFaPhaseDot());
  
  VirtualMechanismGmr<VMP_1ord_t> vm1(test_dim,fa_pos_ptr,fa_phase_dot_ptr);
  VirtualMechanismGmr<VMP_2ord_t> vm2(test_dim,fa_pos_ptr,fa_phase_dot_ptr);
  
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