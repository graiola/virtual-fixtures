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
std::string model_name1 = "test1.txt";
std::string model_name2 = "test2.txt";

TEST(MechanismManagerTest, InitializesCorrectly)
{
  
  ::testing::FLAGS_gtest_death_test_style = "threadsafe"; // NOTE https://code.google.com/p/googletest/wiki/AdvancedGuide#Death_Test_Styles
  
  EXPECT_NO_THROW(MechanismManager());
  
}

TEST(MechanismManagerTest, UpdateMethodWithOrientation)
{
  
  MechanismManager* mm = new MechanismManager();
  
  int pos_dim = mm->GetPositionDim();

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
  EXPECT_NO_THROW(mm->Update(rob_pos,rob_vel,dt,f_out));
  END_REAL_TIME_CRITICAL_CODE();

  delete mm;
}

TEST(MechanismManagerTest, UpdateMethodOnlyPosition)
{
  
  prob_mode_t prob_mode;
  prob_mode = HARD;

  MechanismManager* mm = new MechanismManager();

  int pos_dim = mm->GetPositionDim();
  
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
  EXPECT_NO_THROW(mm->Update(rob_pos,rob_vel,dt,f_out,prob_mode));
  END_REAL_TIME_CRITICAL_CODE();

  delete mm;
}

TEST(MechanismManagerTest, UpdateMethodRawVectors)
{
  MechanismManager* mm = new MechanismManager();

  int pos_dim = mm->GetPositionDim();

  // Force input interface
  std::vector<double> rob_pos(pos_dim, 1.0);
  std::vector<double> rob_vel(pos_dim, 1.0);
  std::vector<double> f_out(pos_dim, 0.0);

  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm->Update(&rob_pos[0],&rob_vel[0],dt,&f_out[0]));
  END_REAL_TIME_CRITICAL_CODE();
}

TEST(MechanismManagerTest, GetVmPositionAndVelocity)
{
  MechanismManager* mm = new MechanismManager();

  int pos_dim = mm->GetPositionDim();

  Eigen::VectorXd pos(pos_dim);
  Eigen::VectorXd vel(pos_dim);
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm->GetVmPosition(0,pos));
  EXPECT_NO_THROW(mm->GetVmVelocity(0,vel));
  END_REAL_TIME_CRITICAL_CODE();

  delete mm;
}

TEST(MechanismManagerTest, InsertVmMethod)
{
  MechanismManager* mm = new MechanismManager();
  EXPECT_NO_THROW(mm->InsertVM());

  delete mm;
}

TEST(MechanismManagerTest, InsertVmUpdateGetPositionAndVelocityDelete) // Most amazing name ever! :)
{
  MechanismManager* mm = new MechanismManager();
  EXPECT_NO_THROW(mm->InsertVM());

  int pos_dim = mm->GetPositionDim();

  // Force input interface
  Eigen::VectorXd rob_pos;
  Eigen::VectorXd rob_vel;
  Eigen::VectorXd f_out;

  rob_pos.resize(pos_dim);
  rob_vel.resize(pos_dim);
  f_out.resize(pos_dim);

  rob_pos << 1.0,2.0;
  rob_vel.fill(1.0);
  f_out.fill(0.0);

  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm->Update(rob_pos,rob_vel,dt,f_out));
  END_REAL_TIME_CRITICAL_CODE();

  Eigen::VectorXd pos(pos_dim);
  Eigen::VectorXd vel(pos_dim);
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm->GetVmPosition(0,pos));
  EXPECT_NO_THROW(mm->GetVmVelocity(0,vel));
  END_REAL_TIME_CRITICAL_CODE();

  EXPECT_NO_THROW(mm->DeleteVM(0));

  delete mm;
}

TEST(MechanismManagerTest, LoopUpdate)
{
  MechanismManager mm;;
  //EXPECT_NO_THROW(mm.InsertVM(model_name1));

  int pos_dim = mm.GetPositionDim();

  // Force input interface
  Eigen::VectorXd rob_pos;
  Eigen::VectorXd rob_vel;
  Eigen::VectorXd f_out;
  Eigen::VectorXd pos(pos_dim);
  Eigen::VectorXd vel(pos_dim);

  rob_pos.resize(pos_dim);
  rob_vel.resize(pos_dim);
  f_out.resize(pos_dim);

  rob_pos << 0.25,0.25;
  rob_vel.fill(1.0);
  f_out.fill(0.0);

  int n_steps = 100000;
  for (int i=0;i<n_steps;i++)
  {

      START_REAL_TIME_CRITICAL_CODE();
      EXPECT_NO_THROW(mm.Update(rob_pos,rob_vel,dt,f_out));
      EXPECT_NO_THROW(mm.GetVmPosition(0,pos));
      EXPECT_NO_THROW(mm.GetVmVelocity(0,vel));
      END_REAL_TIME_CRITICAL_CODE();

      /*if(i == 7)
      {
        std::cout << "DeleteVM " << std::endl;
        EXPECT_NO_THROW(mm.DeleteVM(0));
      }*/
      if (i == 50)
      {
          //std::cout << "InsertVM " << std::endl;
          EXPECT_NO_THROW(mm.InsertVM());
      }

      //std::cout << "Loop cycle: " << i << " of " <<  n_steps << std::endl;
      //getchar();
  }

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
