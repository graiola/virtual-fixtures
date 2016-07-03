#include <toolbox/debug.h>
#include <toolbox/toolbox.h>

#include <gtest/gtest.h>
#include "virtual_mechanism/virtual_mechanism_gmr.h"
#include "virtual_mechanism/virtual_mechanism_spline.h"

////////// Function Approximator
#include <functionapproximators/FunctionApproximatorGMR.hpp>
#include <functionapproximators/ModelParametersGMR.hpp>
#include <functionapproximators/MetaParametersGMR.hpp>

////////// STD
#include <iostream>
#include <fstream> 
#include <iterator>
#include <boost/concept_check.hpp>

using namespace virtual_mechanism_interface;
using namespace virtual_mechanism_gmr;
using namespace virtual_mechanism_spline;
using namespace Eigen;
using namespace boost;
using namespace DmpBbo;

int test_dim = 3;
double dt = 0.001;
double Kf = 100.0;
double Bf = 20.0;
double fade_gain = 10.0;
std::vector<double> K(test_dim,100.0);
std::vector<double> B(test_dim,20.0);

typedef VirtualMechanismInterfaceFirstOrder VMP_1ord_t;
typedef VirtualMechanismInterfaceSecondOrder VMP_2ord_t;

std::string file_name_spline = "/home/sybot/ros_catkin_ws/src/virtual-fixtures/mechanism_manager/models/spline/test_spline1.txt"; // FIXME
std::string file_name_gmr = "/home/sybot/ros_catkin_ws/src/virtual-fixtures/mechanism_manager/models/gmm/test3d.txt"; // FIXME

/*
TEST(VirtualMechanismSplineTest, InitializesCorrectlySpline)
{

  //::testing::FLAGS_gtest_death_test_style = "threadsafe"; // NOTE https://code.google.com/p/googletest/wiki/AdvancedGuide#Death_Test_Styles

  EXPECT_NO_THROW(VirtualMechanismSpline<VMP_1ord_t>(test_dim,K,B,Kf,Bf,fade_gain,file_name_spline));
  EXPECT_NO_THROW(VirtualMechanismSpline<VMP_2ord_t>(test_dim,K,B,Kf,Bf,fade_gain,file_name_spline));
}*/

TEST(VirtualMechanismGmrTest, InitializesCorrectlyGmr)
{

  //ASSERT_DEATH(VirtualMechanismGmr(1,fa_ptr),".*");
  //ASSERT_DEATH(VirtualMechanismGmr(2,fa_ptr),".*");
  EXPECT_NO_THROW(VirtualMechanismGmr<VMP_1ord_t>(test_dim,K,B,Kf,Bf,fade_gain,file_name_gmr));
  EXPECT_NO_THROW(VirtualMechanismGmr<VMP_2ord_t>(test_dim,K,B,Kf,Bf,fade_gain,file_name_gmr));

  EXPECT_NO_THROW(VirtualMechanismGmrNormalized<VMP_1ord_t>(test_dim,K,B,Kf,Bf,fade_gain,file_name_gmr));
  EXPECT_NO_THROW(VirtualMechanismGmrNormalized<VMP_2ord_t>(test_dim,K,B,Kf,Bf,fade_gain,file_name_gmr));

}

TEST(VirtualMechanismGmrTest, UpdateMethodGmr)
{
  VirtualMechanismGmr<VMP_1ord_t> vm1(test_dim,K,B,Kf,Bf,fade_gain,file_name_gmr);
  VirtualMechanismGmr<VMP_2ord_t> vm2(test_dim,K,B,Kf,Bf,fade_gain,file_name_gmr);
  
  Eigen::VectorXd force(test_dim);
  Eigen::VectorXd pos(test_dim);
  Eigen::VectorXd vel(test_dim);
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

TEST(VirtualMechanismGmrNormalizedTest, UpdateMethodGmr)
{
  VirtualMechanismGmrNormalized<VMP_1ord_t> vm1(test_dim,K,B,Kf,Bf,fade_gain,file_name_gmr);
  VirtualMechanismGmrNormalized<VMP_2ord_t> vm2(test_dim,K,B,Kf,Bf,fade_gain,file_name_gmr);

  Eigen::VectorXd force(test_dim);
  Eigen::VectorXd pos(test_dim);
  Eigen::VectorXd vel(test_dim);
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

TEST(VirtualMechanismGmrTest, GetGaussian)
{
  VirtualMechanismGmr<VMP_1ord_t> vm1(test_dim,K,B,Kf,Bf,fade_gain,file_name_gmr);
  VirtualMechanismGmr<VMP_2ord_t> vm2(test_dim,K,B,Kf,Bf,fade_gain,file_name_gmr);
  
  Eigen::VectorXd pos(test_dim);
  pos.fill(1.0);
  
  START_REAL_TIME_CRITICAL_CODE();
  
  EXPECT_NO_THROW(vm1.getGaussian(pos));
  EXPECT_NO_THROW(vm2.getGaussian(pos));
  
  END_REAL_TIME_CRITICAL_CODE();
}

TEST(VirtualMechanismGmrTest, GetDistance)
{
  VirtualMechanismGmr<VMP_1ord_t> vm1(test_dim,K,B,Kf,Bf,fade_gain,file_name_gmr);
  VirtualMechanismGmr<VMP_2ord_t> vm2(test_dim,K,B,Kf,Bf,fade_gain,file_name_gmr);
  
  Eigen::VectorXd pos(test_dim);
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

  VirtualMechanismGmr<VMP_1ord_t> vm1(test_dim,K,B,Kf,Bf,fade_gain,file_name_gmr);
  VirtualMechanismGmr<VMP_2ord_t> vm2(test_dim,K,B,Kf,Bf,fade_gain,file_name_gmr);
  
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



/*
TEST(VirtualMechanismGmrTest, TestGMR)
{
  std::string file_name_input = "/home/sybot/test_data.txt"; // FIXME
  std::string file_name_output = "/home/sybot/test_data_out.txt"; // FIXME


  // Gaussian Mixture Regression (GMR)
  int dim = 2;
  int number_of_gaussians = 15;
  int n_points = 100;
  //bool overwrite = false;
  MetaParametersGMR* meta_parameters_gmr = new MetaParametersGMR(dim,number_of_gaussians);
  FunctionApproximatorGMR* fa_ptr = new FunctionApproximatorGMR(meta_parameters_gmr);
  std::vector<std::vector<double> > data_vector;
  std::vector<std::vector<double> > output_vector(n_points,std::vector<double>(dim));

  tool_box::ReadTxtFile(file_name_input.c_str(),data_vector);

  int nbr = data_vector.size();
  int nbc = data_vector[0].size();

  MatrixXd inputs(nbr,1);
  MatrixXd targets(nbr,dim);
  MatrixXd inputs_predict(n_points,1);
  MatrixXd outputs(n_points,dim);


  for (int i=0;i<nbr;i++)
  {
        inputs(i,0) = data_vector[i][0];
        targets(i,0) = data_vector[i][1];
        targets(i,1) = data_vector[i][2];
  }


  //fa_ptr->train();

  fa_ptr->train(inputs,targets);

  inputs_predict.col(0) = VectorXd::LinSpaced(n_points, 0.0, 1.0);

  fa_ptr->predict(inputs_predict,outputs);

  for (int i=0;i<n_points;i++)
    for (int j=0;j<dim;j++)
        output_vector[i][j] = outputs(i,j);

  tool_box::WriteTxtFile(file_name_output.c_str(),output_vector);


  //VirtualMechanismGmr<VMP_1ord_t> vm1(test_dim,K,B,Kf,Bf,fade_gain,fa_ptr);
  //VirtualMechanismGmr<VMP_2ord_t> vm2(test_dim,K,B,Kf,Bf,fade_gain,fa_ptr);


}
*/


TEST(VirtualMechanismGmrTest, TestGMR)
{
  // Gaussian Mixture Regression (GMR)
  int dim = 2;
  int number_of_gaussians = 10;
  int n_points = 300;
  //bool overwrite = false;
  MetaParametersGMR* meta_parameters_gmr = new MetaParametersGMR(dim,number_of_gaussians);
  FunctionApproximatorGMR* fa_ptr = new FunctionApproximatorGMR(meta_parameters_gmr);
  std::vector<std::vector<double> > data_vector;
  std::vector<std::vector<double> > output_vector(n_points,std::vector<double>(dim));

  std::string file_name_input;
  std::string file_name_output;

  for (int i = 0; i<4; i++)
  {
      file_name_input = "/home/sybot/trj_" + to_string(i+1) + ".txt"; // FIXME
      file_name_output = "/home/sybot/trj_" + to_string(i+1) + "_out.txt"; // FIXME

      tool_box::ReadTxtFile(file_name_input.c_str(),data_vector);

      int nbr = data_vector.size();
      int nbc = data_vector[0].size();

      MatrixXd inputs(nbr,1);
      MatrixXd targets(nbr,nbc);
      MatrixXd inputs_predict(n_points,1);
      MatrixXd outputs(n_points,nbc);

      for (int i=0;i<nbr;i++)
      {
            //inputs(i,0) = data_vector[i][0];
            targets(i,0) = data_vector[i][0];
            targets(i,1) = data_vector[i][1];
      }

      //fa_ptr->train();

      inputs.col(0) = VectorXd::LinSpaced(nbr, 0.0, 1.0);

      fa_ptr->trainIncremental(inputs,targets);

      inputs_predict.col(0) = VectorXd::LinSpaced(n_points, 0.0, 1.0);

      fa_ptr->predict(inputs_predict,outputs);

      for (int i=0;i<n_points;i++)
        for (int j=0;j<dim;j++)
            output_vector[i][j] = outputs(i,j);

      tool_box::WriteTxtFile(file_name_output.c_str(),output_vector);
  }

  delete meta_parameters_gmr;
  delete fa_ptr;

  //VirtualMechanismGmr<VMP_1ord_t> vm1(test_dim,K,B,Kf,Bf,fade_gain,fa_ptr);
  //VirtualMechanismGmr<VMP_2ord_t> vm2(test_dim,K,B,Kf,Bf,fade_gain,fa_ptr);


}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
