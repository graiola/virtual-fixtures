#include <gtest/gtest.h>
#include "virtual_mechanism/virtual_mechanism_gmr.h"

////////// STD
#include <iostream>
#include <fstream> 
#include <iterator>

////////// Function Approximator
#include <functionapproximators/FunctionApproximatorGMR.hpp>
#include <functionapproximators/MetaParametersGMR.hpp>
#include <functionapproximators/ModelParametersGMR.hpp>

using namespace virtual_mechanism_gmr;
using namespace Eigen;
using namespace boost;
using namespace DmpBbo;

int test_dim = 3;
double dt = 0.001;

template<typename value_t>
void ReadTxtFile(const char* filename,std::vector<std::vector<value_t> >& values ) {
    std::string line;
    values.clear();
    std::ifstream myfile (filename);
    std::istringstream iss;
    std::size_t i=0;
    std::size_t nb_vals=0;
    if (myfile.is_open())
    {
        while (getline(myfile,line)) {
            values.push_back(std::vector<value_t>());;
            std::vector<value_t>& v = values[i];
            iss.clear();
            iss.str(line);
            std::copy(std::istream_iterator<value_t>(iss),std::istream_iterator<value_t>(), std::back_inserter(v));
            nb_vals+=v.size();
            i++;
        }
	std::cout << "File ["<<filename<<"] read with success  ["<<nb_vals<<" values, "<<i<<" lines] "<<std::endl;
    }
    else{
	 std::cout << "Unable to open file : ["<<filename<<"]"<<std::endl;
    }
    myfile.close();
}

fa_t* generateDemoFa(){

	// MAKE THE FUNCTION APPROXIMATORS
	int input_dim = 1;
	int n_basis_functions = 25;
	
	// GMR
	MetaParametersGMR* meta_parameters_gmr = new MetaParametersGMR(input_dim,n_basis_functions);
	FunctionApproximatorGMR* fa = new FunctionApproximatorGMR(meta_parameters_gmr);
	
	// Read from file the inputs / targets
	std::vector<std::vector<double> > data;
	std::string file_name = "/home/gennaro/catkin_ws/src/virtual-fixtures/virtual_mechanism/test/01_txyz.txt";
	ReadTxtFile(file_name.c_str(),data);
	
	MatrixXd inputs = VectorXd::LinSpaced(data.size(),0.0,1.0);;
	
	MatrixXd targets(data.size(), data[0].size()-1); // NOTE Skip time
	for (int i = 0; i < data.size(); i++)
	  targets.row(i) = VectorXd::Map(&data[i][1],data[0].size()-1);
	
	//eigen_data = eigen_data.
	
// 	std::cout<<targets<<std::endl;
// 	std::cout<<"******************"<<std::endl;
// 	std::cout<<inputs<<std::endl;
// 	getchar();
	
	//std::cout<<inputs.size()<<std::endl;
	//getchar();
	//std::cout<<targets<<std::endl;
	//getchar();
	
	fa->train(inputs,targets);
	
// 	fa->predict(inputs,targets);
// 	std::cout<<targets<<std::endl;
// 	getchar();
	
	return fa;  
}

TEST(VirtualMechanismGmrTest, InitializesCorrectly)
{
  
  ::testing::FLAGS_gtest_death_test_style = "threadsafe"; // NOTE https://code.google.com/p/googletest/wiki/AdvancedGuide#Death_Test_Styles
  
  boost::shared_ptr<fa_t> fa_ptr(generateDemoFa());
  
  ASSERT_DEATH(VirtualMechanismGmr(1,fa_ptr),".*");
  ASSERT_DEATH(VirtualMechanismGmr(2,fa_ptr),".*");
  EXPECT_NO_THROW(VirtualMechanismGmr(3,fa_ptr));
}

TEST(VirtualMechanismGmrTest, UpdateMethod)
{
  boost::shared_ptr<fa_t> fa_ptr(generateDemoFa());
  
  VirtualMechanismGmr vm(test_dim,fa_ptr);
  
  // Force input interface
  Eigen::VectorXd force;
  force.resize(test_dim);
  //  force << 100.0, 0.0 , 0.0;
  
  EXPECT_NO_THROW(vm.Update(force,dt));
  
  Eigen::VectorXd state;
  state.resize(test_dim);
  
  //   vm.getState(state);
  //   std::cout<<state<<std::endl;
  //   getchar();
  
  // Cart input interface
  Eigen::VectorXd pos;
  pos.resize(test_dim);
  Eigen::VectorXd vel;
  vel.resize(test_dim);
  EXPECT_NO_THROW(vm.Update(pos,vel,dt));
}

/*TEST(VirtualMechanismGmrTest, LoopUpdateMethod)
{
  boost::shared_ptr<fa_t> fa_ptr(generateDemoFa());
  
  VirtualMechanismGmr vm(test_dim,fa_ptr);
  
  // Force input interface
  Eigen::VectorXd force;
  force.resize(test_dim);
  force << 100.0, 0.0 , 0.0;
  
  // State
  Eigen::VectorXd state;
  state.resize(test_dim);
  Eigen::VectorXd state_dot;
  state_dot.resize(test_dim);
  for (int i = 0; i < 500; i++)
  {
    vm.Update(force,dt);
    vm.getState(state);
    vm.getStateDot(state_dot);
    
    std::cout<<"*******"<<std::endl;
    std::cout<<state<<std::endl;
    getchar();
    
  }
  
  EXPECT_NO_THROW(vm.getStateDot(state_dot));
  
}*/

TEST(VirtualMechanismGmrTest, GetMethods)
{
  boost::shared_ptr<fa_t> fa_ptr(generateDemoFa());
  
  VirtualMechanismGmr vm(test_dim,fa_ptr);
  
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