#include <toolbox/debug.h>

#include <gtest/gtest.h>
#include "mechanism_manager/mechanism_manager_interface.h"

////////// STD
#include <iostream>
#include <fstream> 
#include <iterator>
#include <boost/concept_check.hpp>

using namespace mechanism_manager;
using namespace Eigen;
using namespace boost;

double dt = 0.001;
std::string model_name = "test_gmm.txt";
std::string model_name_wrong = "wrong.txt";

TEST(MechanismManagerTest, InitializesCorrectly)
{
  
  ::testing::FLAGS_gtest_death_test_style = "threadsafe"; // NOTE https://code.google.com/p/googletest/wiki/AdvancedGuide#Death_Test_Styles
  
  EXPECT_NO_THROW(MechanismManagerInterface());
  
}

TEST(MechanismManagerTest, UpdateMethodOnlyPosition)
{

  scale_mode_t scale_mode;
  scale_mode = HARD;
  MechanismManagerInterface* mm = new MechanismManagerInterface();

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
  EXPECT_NO_THROW(mm->Update(rob_pos,rob_vel,dt,f_out,scale_mode));
  END_REAL_TIME_CRITICAL_CODE();

  delete mm;
}

TEST(MechanismManagerTest, UpdateMethodRawVectors)
{
  MechanismManagerInterface* mm = new MechanismManagerInterface();

  int pos_dim = mm->GetPositionDim();

  // Force input interface
  std::vector<double> rob_pos_std(pos_dim, 1.0);
  std::vector<double> rob_vel_std(pos_dim, 1.0);
  std::vector<double> f_out_std(pos_dim, 0.0);

  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm->Update(&rob_pos_std[0],&rob_vel_std[0],dt,&f_out_std[0]));
  END_REAL_TIME_CRITICAL_CODE();

  delete mm;
}

TEST(MechanismManagerTest, GetVmPositionAndVelocity)
{
  MechanismManagerInterface* mm = new MechanismManagerInterface();

  int pos_dim = mm->GetPositionDim();

  Eigen::VectorXd pos(pos_dim);
  Eigen::VectorXd vel(pos_dim);
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm->GetVmPosition(0,pos));
  EXPECT_NO_THROW(mm->GetVmVelocity(0,vel));
  END_REAL_TIME_CRITICAL_CODE();

  delete mm;
}

TEST(MechanismManagerTest, GetVmPositionAndVelocityRawVectors)
{
  MechanismManagerInterface* mm = new MechanismManagerInterface();

  int pos_dim = mm->GetPositionDim();

  std::vector<double> pos_std(pos_dim);
  std::vector<double> vel_std(pos_dim);

  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm->GetVmPosition(0,&pos_std[0]));
  EXPECT_NO_THROW(mm->GetVmVelocity(0,&vel_std[0]));
  END_REAL_TIME_CRITICAL_CODE();

  delete mm;
}

TEST(MechanismManagerTest, InsertVmMethod)
{
  MechanismManagerInterface* mm = new MechanismManagerInterface();
  EXPECT_NO_THROW(mm->InsertVm(model_name));

  delete mm;
}

TEST(MechanismManagerTest, SaveVmMethod)
{
  MechanismManagerInterface* mm = new MechanismManagerInterface();

  EXPECT_NO_THROW(mm->InsertVm(model_name));

  std::cout << "Press to continue..." << std::endl;
  getchar();

  EXPECT_NO_THROW(mm->SaveVm(0));

  std::cout << "Press to continue..." << std::endl;
  getchar();

  delete mm;
}

TEST(MechanismManagerTest, InsertVmUpdateGetPositionAndVelocityDelete) // Most amazing name ever! :)
{
  MechanismManagerInterface mm;

  // Insert
  EXPECT_NO_THROW(mm.InsertVm(model_name));

  int pos_dim = mm.GetPositionDim();

  // EIGEN INTERFACE
  Eigen::VectorXd rob_pos;
  Eigen::VectorXd rob_vel;
  Eigen::VectorXd f_out;
  Eigen::VectorXd pos(pos_dim);
  Eigen::VectorXd vel(pos_dim);
  rob_pos.resize(pos_dim);
  rob_vel.resize(pos_dim);
  f_out.resize(pos_dim);
  rob_pos.fill(2.0);
  rob_vel.fill(1.0);
  f_out.fill(0.0);
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm.Update(rob_pos,rob_vel,dt,f_out)); // Update
  EXPECT_NO_THROW(mm.GetVmPosition(0,pos)); // Get
  EXPECT_NO_THROW(mm.GetVmVelocity(0,vel)); // Get
  END_REAL_TIME_CRITICAL_CODE();

  // RAW VECTORS INTERFACE
  std::vector<double> rob_pos_std(pos_dim, 1.0);
  std::vector<double> rob_vel_std(pos_dim, 1.0);
  std::vector<double> f_out_std(pos_dim, 0.0);
  std::vector<double> pos_std(pos_dim);
  std::vector<double> vel_std(pos_dim);
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(mm.Update(&rob_pos_std[0],&rob_vel_std[0],dt,&f_out_std[0])); // Update
  EXPECT_NO_THROW(mm.GetVmPosition(0,&pos_std[0])); // Get
  EXPECT_NO_THROW(mm.GetVmVelocity(0,&vel_std[0])); // Get
  END_REAL_TIME_CRITICAL_CODE();

  std::cout << "Press to continue..." << std::endl;
  getchar();

  // Delete Note: this is async, so it could happen that there is nothing to delete because Insert is still going on
  EXPECT_NO_THROW(mm.DeleteVm(0));

  std::cout << "Press to continue..." << std::endl;
  getchar();

}

TEST(MechanismManagerTest, LoopUpdate)
{
  //int nb = omp_get_num_threads();
  //std::cout << "OMP THREADS = " << nb <<std::endl;

  //Eigen::setNbThreads(4);
  //int nthreads = Eigen::nbThreads( );
  //std::cout << "EIGEN THREADS = " << nthreads <<std::endl;

  MechanismManagerInterface mm;
  EXPECT_NO_THROW(mm.InsertVm(model_name));

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

  rob_pos.fill(0.25);
  rob_vel.fill(1.0);
  f_out.fill(0.0);

  double scale, phase;

  long long loopCnt = 0;

  int n_steps = 1500; // Give enough time to test stuff
  for (int i=0;i<n_steps;i++)
  {

      //if(loopCnt%500==0)
      //    mm.SaveVm(0);

      loopCnt++;

      //START_REAL_TIME_CRITICAL_CODE();
      EXPECT_NO_THROW(mm.Update(rob_pos,rob_vel,dt,f_out));
      EXPECT_NO_THROW(mm.GetVmPosition(0,pos));
      EXPECT_NO_THROW(mm.GetVmVelocity(0,vel));
      EXPECT_NO_THROW(scale = mm.GetScale(0));
      EXPECT_NO_THROW(phase = mm.GetPhase(0));
      //END_REAL_TIME_CRITICAL_CODE();

      if(i == 10)
      {
        EXPECT_NO_THROW(mm.DeleteVm(0));
      }
      if (i == 100)
      {
          EXPECT_NO_THROW(mm.InsertVm(model_name));
      }

      if (i == 300)
      {
          EXPECT_NO_THROW(mm.InsertVm(model_name));
      }

      if (i == 400)
      {
          EXPECT_NO_THROW(mm.InsertVm(model_name_wrong));
      }

      //std::cout << "Loop cycle: " << i << " of " <<  n_steps << std::endl;
  }

  std::cout << "Press to continue..." << std::endl;
  getchar();
}

int main(int argc, char** argv)
{
  //Eigen::initParallel();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
