#include <gtest/gtest.h>
#include "mechanism_manager/mechanism_manager.h"

////////// STD
#include <iostream>
#include <fstream> 
#include <iterator>
#include <boost/concept_check.hpp>

////////// Function Approximator
#include <functionapproximators/FunctionApproximatorGMR.hpp>
#include <functionapproximators/MetaParametersGMR.hpp>
#include <functionapproximators/ModelParametersGMR.hpp>

using namespace mechanism_manager;
using namespace Eigen;
using namespace boost;
using namespace DmpBbo;

int test_dim = 3;
double dt = 0.001;

TEST(VirtualMechanismGmrTest, InitializesCorrectly)
{
  
  ::testing::FLAGS_gtest_death_test_style = "threadsafe"; // NOTE https://code.google.com/p/googletest/wiki/AdvancedGuide#Death_Test_Styles
  
  EXPECT_NO_THROW(MechanismManager());
  
}

TEST(VirtualMechanismGmrTest, UpdateMethod)
{
  
  MechanismManager mm = MechanismManager();
  
  // Force input interface
  Eigen::VectorXd rob_pos;
  Eigen::VectorXd rob_vel;
  Eigen::VectorXd f_out;
  
  rob_pos.resize(test_dim);
  rob_vel.resize(test_dim);
  f_out.resize(test_dim);
  
  rob_pos.fill(1.0);
  rob_vel.fill(1.0);
  f_out.fill(0.0);

  EXPECT_NO_THROW(mm.Update(rob_pos,rob_vel,dt,f_out));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}