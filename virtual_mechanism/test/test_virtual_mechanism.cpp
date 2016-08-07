#include <gtest/gtest.h>
#include "virtual_mechanism/virtual_mechanism_interface.h"
#include "virtual_mechanism/virtual_mechanism_factory.h"

using namespace virtual_mechanism_factory;

std::string pkg_path = ros::package::getPath("virtual_mechanism");
std::string file_path(pkg_path+"/test/test_gmm.txt");

order_t order;
model_type_t model_type;
VirtualMechanismFactory vm_factory;


TEST(VirtualMechanismFactory, BuildFromFile)
{
    VirtualMechanismInterface* vm_ptr = NULL;

    // FROM FILE
    order = FIRST;
    model_type = GMR;
    EXPECT_NO_THROW(vm_ptr = vm_factory.Build(order,model_type,file_path));
    order = SECOND;
    model_type = GMR;
    EXPECT_NO_THROW(vm_ptr = vm_factory.Build(order,model_type,file_path));
    order = FIRST;
    model_type = GMR_NORMALIZED;
    EXPECT_NO_THROW(vm_ptr = vm_factory.Build(order,model_type,file_path));
    order = SECOND;
    model_type = GMR_NORMALIZED;
    EXPECT_NO_THROW(vm_ptr = vm_factory.Build(order,model_type,file_path));

    delete vm_ptr;
}

TEST(VirtualMechanismFactory, BuildFromData)
{
    VirtualMechanismInterface* vm_ptr = NULL;

    int n_points = 50;
    int test_dim = 2;
    Eigen::MatrixXd data(n_points,test_dim); // No phase

    for (int i=0; i<data.cols(); i++)
        data.col(i) = Eigen::VectorXd::LinSpaced(n_points, 0.0, 1.0);

    // FROM DATA
    order = FIRST;
    model_type = GMR;
    EXPECT_NO_THROW(vm_ptr = vm_factory.Build(order,model_type,data));
    order = SECOND;
    model_type = GMR;
    EXPECT_NO_THROW(vm_ptr = vm_factory.Build(order,model_type,data));
    order = FIRST;
    model_type = GMR_NORMALIZED;
    EXPECT_NO_THROW(vm_ptr = vm_factory.Build(order,model_type,data));
    order = SECOND;
    model_type = GMR_NORMALIZED;
    EXPECT_NO_THROW(vm_ptr = vm_factory.Build(order,model_type,data));

    /*double dt = 0.01;
    Eigen::VectorXd force(test_dim);
    Eigen::VectorXd pos(test_dim);
    Eigen::VectorXd vel(test_dim);
    force.fill(1.0);

    vm_ptr->Update(pos,vel,dt);

    Eigen::VectorXd state(test_dim);
    vm_ptr->getState(state);

    std::cout << state << std::endl;*/

    delete vm_ptr;
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


  //VirtualMechanismGmr<VMP_1ord_t> vm1(test_dim,K,B,Kf,Bf,fade_gain,fa_ptr);
  //VirtualMechanismGmr<VMP_2ord_t> vm2(test_dim,K,B,Kf,Bf,fade_gain,fa_ptr);


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

  //VirtualMechanismGmr<VMP_1ord_t> vm1(test_dim,K,B,Kf,Bf,fade_gain,fa_ptr);
  //VirtualMechanismGmr<VMP_2ord_t> vm2(test_dim,K,B,Kf,Bf,fade_gain,fa_ptr);
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
