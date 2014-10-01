#include <gtest/gtest.h>
#include "virtual_mechanism/virtual_mechanism_line.h"

using namespace virtual_mechanism_line;
using namespace Eigen;

int test_dim = 3;
double dt = 0.001;

VectorXd Pf;
VectorXd Pi;

TEST(VirtualMechanismLineTest, InitializesCorrectly)
{	
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  
  Pi.resize(test_dim,1);
  Pf.resize(test_dim,1);

  Pi << 0.3, -0.22, -0.179; 
  Pf << 0.5, -0.22, -0.179;
  
  ASSERT_DEATH(VirtualMechanismLine(1,Pf,Pi),".*");
  ASSERT_DEATH(VirtualMechanismLine(2,Pf,Pi),".*");
  EXPECT_NO_THROW(VirtualMechanismLine(3,Pf,Pi));
}

TEST(VirtualMechanismLineTest, UpdateMethod)
{
  
  Pi.resize(test_dim,1);
  Pf.resize(test_dim,1);

  Pi << 0.3, -0.22, -0.179; 
  Pf << 0.5, -0.22, -0.179;
  
  VirtualMechanismLine vm(test_dim,Pf,Pi);
  
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

TEST(VirtualMechanismLineTest, GetMethods)
{
  Pi.resize(test_dim,1);
  Pf.resize(test_dim,1);

  Pi << 0.3, -0.22, -0.179; 
  Pf << 0.5, -0.22, -0.179;
  
  VirtualMechanismLine vm(test_dim,Pf,Pi);
  
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