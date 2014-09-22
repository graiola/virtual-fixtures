#include <gtest/gtest.h>
#include "virtual_mechanism/virtual_mechanism.h"

using namespace virtual_mechanism;
using namespace Eigen;

int test_dim = 3;
double dt = 0.001;

TEST(VirtualMechanismTest, InitializesCorrectly)
{
  
  ASSERT_DEATH(VirtualMechanism(1),".*");
  EXPECT_NO_THROW(VirtualMechanism(2));
  EXPECT_NO_THROW(VirtualMechanism(3));
}

TEST(VirtualMechanismTest, UpdateMethod)
{
  VirtualMechanism vm(test_dim);
  
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
  VirtualMechanism vm(test_dim);
  
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