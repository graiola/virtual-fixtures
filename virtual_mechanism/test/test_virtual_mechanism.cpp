#include <gtest/gtest.h>
#include "virtual_mechanism/virtual_mechanism.h"

using namespace virtual_mechanism;
using namespace Eigen;

int test_dim = 3;
double dt = 0.01;


TEST(VirtualMechanismTest, InitializesCorrectly)
{
  
  Eigen::VectorXd Pf(1);
  Eigen::VectorXd Pi(1);
  
  ASSERT_DEATH(VirtualMechanism(1,Pf,Pi),".*");
  
  Pf.resize(2);
  Pi.resize(2);
  EXPECT_NO_THROW(VirtualMechanism(2,Pf,Pi));
  
  Pf.resize(3);
  Pi.resize(3);
  EXPECT_NO_THROW(VirtualMechanism(3,Pf,Pi));
}

TEST(VirtualMechanismTest, UpdateMethod)
{
   
  Eigen::VectorXd Pf(test_dim);
  Eigen::VectorXd Pi(test_dim);
  
  VirtualMechanism vm(test_dim,Pf,Pi);
  
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



TEST(VirtualMechanismTest, GetMethods)
{
  Eigen::VectorXd Pf(test_dim);
  Eigen::VectorXd Pi(test_dim);
  
  VirtualMechanism vm(test_dim,Pf,Pi);
  
  // State
  Eigen::VectorXd state;
  state.resize(test_dim);
  EXPECT_NO_THROW(vm.getState(state));
  
  // StateDot
  Eigen::VectorXd state_dot;
  state_dot.resize(test_dim);
  EXPECT_NO_THROW(vm.getStateDot(state_dot));
}

TEST(VirtualMechanismTest, UpdateMethodRun)
{
   
  Eigen::VectorXd Pf(test_dim);
  Eigen::VectorXd Pi(test_dim);
  
  Pi << 0.0, 0.0, -0.1;
  Pf << 1.0, -0.2, -0.1;
  
  VirtualMechanism vm(test_dim,Pf,Pi);
  
  // Force input interface
  Eigen::VectorXd force, state;
  force.resize(test_dim);
  state.resize(test_dim);
  
  force << 10.0, 0.0, 0.0;
  
  for(int i=1;i<500;i++)
  {
      vm.Update(force,dt);
      vm.getState(state);
      std::cout<<"** STATE **"<<std::endl;
      std::cout<<state<<std::endl;
      std::cout<<"** PHASE **"<<std::endl;
      std::cout<<vm.getPhase()<<std::endl;
      getchar();
  }

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}