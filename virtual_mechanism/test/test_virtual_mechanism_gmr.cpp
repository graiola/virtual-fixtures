/**
 * @file   test_virtual_mechanism_gmr.cpp
 * @brief  GTest.
 * @author Gennaro Raiola
 *
 * This file is part of virtual-fixtures, a set of libraries and programs to create
 * and interact with a library of virtual guides.
 * Copyright (C) 2014-2016 Gennaro Raiola, ENSTA-ParisTech
 *
 * virtual-fixtures is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * virtual-fixtures is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with virtual-fixtures.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <toolbox/debug.h>
#include <toolbox/toolbox.h>
#include <toolbox/dtw/dtw.h>

#include <gtest/gtest.h>
#include "virtual_mechanism/virtual_mechanism_gmr.h"

////////// Function Approximator
#include <functionapproximators/FunctionApproximatorGMR.hpp>
#include <functionapproximators/ModelParametersGMR.hpp>
#include <functionapproximators/MetaParametersGMR.hpp>

////////// STD
#include <iostream>
#include <fstream> 
#include <iterator>
#include <boost/concept_check.hpp>

////////// ROS
#include <ros/ros.h>
#include <ros/package.h>

using namespace virtual_mechanism;
using namespace Eigen;
using namespace boost;
using namespace DmpBbo;

typedef VirtualMechanismInterfaceFirstOrder VMP_1ord_t;
typedef VirtualMechanismInterfaceSecondOrder VMP_2ord_t;

std::string pkg_path = ros::package::getPath("virtual_mechanism");
std::string file_path(pkg_path+"/test/test_gmm");
double dt = 0.001;
int test_dim = 2;

TEST(VirtualMechanismGmrTest, InitializesCorrectlyFromFile)
{
  EXPECT_NO_THROW(VirtualMechanismGmr<VMP_1ord_t> vm1(file_path));
  EXPECT_NO_THROW(VirtualMechanismGmr<VMP_2ord_t> vm2(file_path));

  EXPECT_NO_THROW(VirtualMechanismGmrNormalized<VMP_1ord_t> vm1(file_path));
  EXPECT_NO_THROW(VirtualMechanismGmrNormalized<VMP_2ord_t> vm2(file_path));
}

TEST(VirtualMechanismGmrTest, InitializesCorrectlyFromData)
{
  int n_points = 50;
  MatrixXd data(n_points,test_dim); // No phase

  for (int i=0; i<data.cols(); i++)
      data.col(i) = VectorXd::LinSpaced(n_points, 0.0, 1.0);

  EXPECT_NO_THROW(VirtualMechanismGmr<VMP_1ord_t> vm1(data));
  EXPECT_NO_THROW(VirtualMechanismGmr<VMP_2ord_t> vm2(data));

  EXPECT_NO_THROW(VirtualMechanismGmrNormalized<VMP_1ord_t> vm1(data));
  EXPECT_NO_THROW(VirtualMechanismGmrNormalized<VMP_2ord_t> vm2(data));

  data.resize(n_points,test_dim+1); // With phase

  for (int i=0; i<data.cols(); i++)
      data.col(i) = VectorXd::LinSpaced(n_points, 0.0, 1.0);

  EXPECT_NO_THROW(VirtualMechanismGmr<VMP_1ord_t> vm1(data));
  EXPECT_NO_THROW(VirtualMechanismGmr<VMP_2ord_t> vm2(data));

  EXPECT_NO_THROW(VirtualMechanismGmrNormalized<VMP_1ord_t> vm1(data));
  EXPECT_NO_THROW(VirtualMechanismGmrNormalized<VMP_2ord_t> vm2(data));
}

TEST(VirtualMechanismGmrTest, UpdateMethod)
{
  VirtualMechanismGmr<VMP_1ord_t> vm1(file_path);
  VirtualMechanismGmr<VMP_2ord_t> vm2(file_path);
  
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

TEST(VirtualMechanismGmrTest, GetScale)
{
  VirtualMechanismGmr<VMP_1ord_t> vm1(file_path);
  VirtualMechanismGmr<VMP_2ord_t> vm2(file_path);

  Eigen::VectorXd pos(test_dim);
  pos.fill(1.0);

  START_REAL_TIME_CRITICAL_CODE();

  EXPECT_NO_THROW(vm1.getScale(pos));
  EXPECT_NO_THROW(vm2.getScale(pos));

  END_REAL_TIME_CRITICAL_CODE();
}

TEST(VirtualMechanismGmrTest, GetMethods)
{

  VirtualMechanismGmr<VMP_1ord_t> vm1(file_path);
  VirtualMechanismGmr<VMP_2ord_t> vm2(file_path);

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

TEST(VirtualMechanismGmrNormalizedTest, UpdateMethod)
{
  VirtualMechanismGmrNormalized<VMP_1ord_t> vm1(file_path);
  VirtualMechanismGmrNormalized<VMP_2ord_t> vm2(file_path);

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

/*TEST(VirtualMechanismGmrTest, UpdateGuideNormalized)
{
  int n_points = 100;
  MatrixXd data = MatrixXd::Random(n_points,test_dim); // No phase

  //for (int i=0; i<data.cols(); i++)
  //    data.col(i) = VectorXd::LinSpaced(n_points, 0.0, 1.0);

  VirtualMechanismGmrNormalized<VMP_1ord_t> vm1(data);
  VirtualMechanismGmrNormalized<VMP_2ord_t> vm2(data);

  EXPECT_NO_THROW(vm1.UpdateGuide(data));
  EXPECT_NO_THROW(vm2.UpdateGuide(data));

  data.resize(n_points,test_dim+1); // With phase

  for (int i=0; i<data.cols(); i++)
      data.col(i) = VectorXd::LinSpaced(n_points, 0.0, 1.0);

  VirtualMechanismGmrNormalized<VMP_1ord_t> vm1(data);
  VirtualMechanismGmrNormalized<VMP_2ord_t> vm2(data);

  EXPECT_NO_THROW(vm1.UpdateGuide(data));
  EXPECT_NO_THROW(vm2.UpdateGuide(data));
}*/

/*TEST(VirtualMechanismGmrTest, GetGaussian)
{
  VirtualMechanismGmr<VMP_1ord_t> vm1(file_name_gmr);
  VirtualMechanismGmr<VMP_2ord_t> vm2(file_name_gmr);
  
  Eigen::VectorXd pos(test_dim);
  pos.fill(1.0);
  
  START_REAL_TIME_CRITICAL_CODE();
  
  EXPECT_NO_THROW(vm1.getGaussian(pos));
  EXPECT_NO_THROW(vm2.getGaussian(pos));
  
  END_REAL_TIME_CRITICAL_CODE();
}*/




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


/*
TEST(VirtualMechanismGmrTest, TestDtw)
{
    //int n_points = 10;
    //MatrixXd data1 = MatrixXd::Random(10,1);
    //MatrixXd data2 = MatrixXd::Random(5,1);

    //MatrixXd phase1 = VectorXd::LinSpaced(10, 0.0, 1.0);
    //VectorXd phase2 = VectorXd::LinSpaced(5, 0.0, 1.0);

    std::vector<std::vector<double> > data1_vector, data2_vector;
    std::vector<double> phase_vector;

    std::string data1_name, data2_name, name_output;

    data1_name = "/media/sf_v3d/trj_1.txt"; // FIXME
    data2_name = "/media/sf_v3d/trj_2.txt"; // FIXME

    name_output = "/media/sf_v3d/dtw_out.txt";

    tool_box::ReadTxtFile(data1_name.c_str(),data1_vector);
    tool_box::ReadTxtFile(data2_name.c_str(),data2_vector);

    int nbr1 = data1_vector.size();
    int nbc1 = data1_vector[0].size();

    int nbr2 = data2_vector.size();
    int nbc2 = data2_vector[0].size();

    MatrixXd phase1(nbr1,1);
    MatrixXd data1(nbr1,nbc1);

    MatrixXd phase2(nbr2,1);
    MatrixXd data2(nbr2,nbc2);

    for (int i=0;i<nbr1;i++)
    {
          data1(i,0) = data1_vector[i][0];
          data1(i,1) = data1_vector[i][1];
    }

    for (int i=0;i<nbr2;i++)
    {
          data2(i,0) = data2_vector[i][0];
          data2(i,1) = data2_vector[i][1];
    }

    phase1.col(0) = VectorXd::LinSpaced(nbr1, 0.0, 1.0);
    phase2.col(0) = VectorXd::LinSpaced(nbr2, 0.0, 1.0);

    EXPECT_NO_THROW(dtw::align_phase(phase1,phase2,data1,data2));

    phase_vector.resize(nbr1);

    for (int j=0;j<nbr1;j++)
      phase_vector[j] = phase1(j,0);

    tool_box::WriteTxtFile(name_output.c_str(),phase_vector);
}
*/

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


  //VirtualMechanismGmr<VMP_1ord_t> vm1(fa_ptr);
  //VirtualMechanismGmr<VMP_2ord_t> vm2(fa_ptr);


}
*/

/*
TEST(VirtualMechanismGmrTest, TestGMR)
{
  // Gaussian Mixture Regression (GMR)
  int dim_out = 2;
  int dim_in = 1;
  int number_of_gaussians = 20;
  int n_points = 300;
  //bool overwrite = false;
  MetaParametersGMR* meta_parameters_gmr = new MetaParametersGMR(dim_in,number_of_gaussians);
  FunctionApproximatorGMR* fa_ptr = new FunctionApproximatorGMR(meta_parameters_gmr);
  std::vector<std::vector<double> > data_vector;
  std::vector<std::vector<double> > output_vector(n_points,std::vector<double>(dim_out));

  std::string file_name_input;
  std::string file_name_output;
  std::string gmm_name_output;

  MatrixXd inputs, targets, inputs_predict, outputs;

  for (int i = 0; i<4; i++)
  {
      file_name_input = "/media/sf_v3d/trj_" + to_string(i+1) + ".txt"; // FIXME
      file_name_output = "/media/sf_v3d/trj_" + to_string(i+1) + "_out.txt";
      gmm_name_output = "/media/sf_v3d/gmm_" + to_string(i+1) + "_out.txt";

      tool_box::ReadTxtFile(file_name_input.c_str(),data_vector);

      int nbr = data_vector.size();
      int nbc = data_vector[0].size();

      inputs.resize(nbr,1);
      targets.resize(nbr,nbc);
      inputs_predict.resize(n_points,1);
      outputs.resize(n_points,nbc);

      for (int i=0;i<nbr;i++)
      {
            //inputs(i,0) = data_vector[i][0];
            targets(i,0) = data_vector[i][0];
            targets(i,1) = data_vector[i][1];
      }

      //fa_ptr->train();

      inputs.col(0) = VectorXd::LinSpaced(nbr, 0.0, 1.0);

      fa_ptr->trainIncremental(inputs,targets);

      const ModelParametersGMR* model_parameters_GMR = static_cast<const ModelParametersGMR*>(fa_ptr->getModelParameters());
      model_parameters_GMR->saveGMMToMatrix(gmm_name_output, true);

      inputs_predict.col(0) = VectorXd::LinSpaced(n_points, 0.0, 1.0);

      fa_ptr->predict(inputs_predict,outputs);

      for (int i=0;i<n_points;i++)
        for (int j=0;j<dim_out;j++)
            output_vector[i][j] = outputs(i,j);

      tool_box::WriteTxtFile(file_name_output.c_str(),output_vector);
  }

  delete meta_parameters_gmr;
  delete fa_ptr;

  // Load from txt

  ModelParametersGMR* model_parameters_gmr = ModelParametersGMR::loadGMMFromMatrix(gmm_name_output);
  FunctionApproximatorGMR* fa_ptr_new = new fa_t(model_parameters_gmr);
  fa_ptr_new->trainIncremental(inputs,targets);

  delete fa_ptr_new;

  //VirtualMechanismGmr<VMP_1ord_t> vm1(fa_ptr);
  //VirtualMechanismGmr<VMP_2ord_t> vm2(fa_ptr);
}

TEST(VirtualMechanismGmrTest, TestGMRLoadAndSave)
{
    // Gaussian Mixture Regression (GMR)
    std::string gmm_name_input = "/media/sf_v3d/test_gmm.txt";
    std::string gmm_name_output = "/media/sf_v3d/test_gmm_out.txt";

    // LOAD
    ModelParametersGMR* model_parameters_gmr = ModelParametersGMR::loadGMMFromMatrix(gmm_name_input);
    FunctionApproximatorGMR* fa_ptr = new fa_t(model_parameters_gmr);

    // SAVE
    //const ModelParametersGMR* model_parameters_gmr = static_cast<const ModelParametersGMR*>(fa_ptr->getModelParameters());
    model_parameters_gmr->saveGMMToMatrix(gmm_name_output, true);
}
*/
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
