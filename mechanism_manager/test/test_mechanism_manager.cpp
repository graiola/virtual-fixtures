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

TEST(MechanismManagerTest, InitializesCorrectly)
{
  
  ::testing::FLAGS_gtest_death_test_style = "threadsafe"; // NOTE https://code.google.com/p/googletest/wiki/AdvancedGuide#Death_Test_Styles
  
  EXPECT_NO_THROW(MechanismManager());
  
}

TEST(MechanismManagerTest, UpdateMethodWithOrientation)
{
  
  MechanismManager mm = MechanismManager();
  
  int pos_dim = mm.GetPositionDim();

  // Force input interface
  Eigen::VectorXd rob_pos;
  Eigen::VectorXd rob_vel;
  Eigen::VectorXd f_out;
  
  rob_pos.resize(pos_dim+4); // position dim + orientation dim
  rob_vel.resize(pos_dim);
  f_out.resize(pos_dim+3); // position dim  + rpy
  
  rob_pos.fill(1.0);
  rob_vel.fill(1.0);
  f_out.fill(0.0);

  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm.Update(rob_pos,rob_vel,dt,f_out));
  END_REAL_TIME_CRITICAL_CODE();
}

TEST(MechanismManagerTest, UpdateMethodOnlyPosition)
{
  
  MechanismManager mm = MechanismManager();

  int pos_dim = mm.GetPositionDim();
  
  // Force input interface
  Eigen::VectorXd rob_pos;
  Eigen::VectorXd rob_vel;
  Eigen::VectorXd f_out;
  
  rob_pos.resize(pos_dim);
  rob_vel.resize(pos_dim);
  f_out.resize(pos_dim);
  
  rob_pos.fill(1.0);
  rob_vel.fill(1.0);
  f_out.fill(0.0);
  
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm.Update(rob_pos,rob_vel,dt,f_out));
  END_REAL_TIME_CRITICAL_CODE();


  /*Eigen::VectorXd pos(pos_dim);
  Eigen::VectorXd vel(pos_dim);
  mm.GetVmPosition(0,pos);
  mm.GetVmVelocity(0,vel);

  std::cout << "f_out" <<std::endl;
  std::cout << f_out <<std::endl;
  std::cout << "pos" <<std::endl;
  std::cout << pos <<std::endl;
  std::cout << "vel" <<std::endl;
  std::cout << vel <<std::endl;*/
}

TEST(MechanismManagerTest, UpdateMethodRawVectors)
{

  MechanismManager mm = MechanismManager();

  int pos_dim = mm.GetPositionDim();

  // Force input interface
  std::vector<double> rob_pos(pos_dim, 1.0);
  std::vector<double> rob_vel(pos_dim, 1.0);
  std::vector<double> f_out(pos_dim, 0.0);

  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm.Update(&rob_pos[0],&rob_vel[0],dt,&f_out[0]));
  END_REAL_TIME_CRITICAL_CODE();

}

TEST(MechanismManagerTest, GetVmPositionVelocity)
{
  MechanismManager mm = MechanismManager();

  int pos_dim = mm.GetPositionDim();

  Eigen::VectorXd pos(pos_dim);
  Eigen::VectorXd vel(pos_dim);
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm.GetVmPosition(0,pos));
  EXPECT_NO_THROW(mm.GetVmVelocity(0,vel));
  END_REAL_TIME_CRITICAL_CODE();

  /*std::cout << "f_out" <<std::endl;
  std::cout << f_out <<std::endl;
  std::cout << "pos" <<std::endl;
  std::cout << pos <<std::endl;
  std::cout << "vel" <<std::endl;
  std::cout << vel <<std::endl;*/
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
