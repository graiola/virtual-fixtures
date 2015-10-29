#include <toolbox/debug.h>

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

double dt = 0.001;

TEST(VirtualMechanismGmrTest, InitializesCorrectly)
{
  
  ::testing::FLAGS_gtest_death_test_style = "threadsafe"; // NOTE https://code.google.com/p/googletest/wiki/AdvancedGuide#Death_Test_Styles
  
  EXPECT_NO_THROW(MechanismManager());
  
}

TEST(VirtualMechanismGmrTest, UpdateMethodWithOrientation)
{
  
  MechanismManager mm = MechanismManager();
  
  // Force input interface
  Eigen::VectorXd rob_pos;
  Eigen::VectorXd rob_vel;
  Eigen::VectorXd f_out;
  
  rob_pos.resize(7);
  rob_vel.resize(3);
  f_out.resize(6);
  
  rob_pos.fill(1.0);
  rob_vel.fill(1.0);
  f_out.fill(0.0);
  
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm.Update(rob_pos,rob_vel,dt,f_out));
  END_REAL_TIME_CRITICAL_CODE();
}

TEST(VirtualMechanismGmrTest, UpdateMethodOnlyPosition)
{
  
  MechanismManager mm = MechanismManager();
  
  // Force input interface
  Eigen::VectorXd rob_pos;
  Eigen::VectorXd rob_vel;
  Eigen::VectorXd f_out;
  
  rob_pos.resize(3);
  rob_vel.resize(3);
  f_out.resize(3);
  
  rob_pos.fill(1.0);
  rob_vel.fill(1.0);
  f_out.fill(0.0);
  
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm.Update(rob_pos,rob_vel,dt,f_out));
  END_REAL_TIME_CRITICAL_CODE();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}