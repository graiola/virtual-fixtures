#include <toolbox/debug.h>
#include <toolbox/toolbox.h>
#include <toolbox/dtw/dtw.h>

#include <gtest/gtest.h>
#include "virtual_mechanism/virtual_mechanism_spline.h"

////////// STD
#include <iostream>
#include <fstream>
#include <iterator>
#include <boost/concept_check.hpp>

////////// ROS
#include <ros/ros.h>
#include <ros/package.h>

using namespace virtual_mechanism_interface;
using namespace virtual_mechanism_spline;
using namespace Eigen;
using namespace boost;

typedef VirtualMechanismInterfaceFirstOrder VMP_1ord_t;
typedef VirtualMechanismInterfaceSecondOrder VMP_2ord_t;

std::string pkg_path = ros::package::getPath("virtual_mechanism");
std::string file_path(pkg_path+"/test/test_spline.txt");

TEST(VirtualMechanismSplineTest, InitializesCorrectlySpline)
{

  //::testing::FLAGS_gtest_death_test_style = "threadsafe"; // NOTE https://code.google.com/p/googletest/wiki/AdvancedGuide#Death_Test_Styles

  EXPECT_NO_THROW(VirtualMechanismSpline<VMP_1ord_t> vm1(file_path));
  EXPECT_NO_THROW(VirtualMechanismSpline<VMP_2ord_t> vm2(file_path));
}

/*TEST(VirtualMechanismSplineTest, InitializesCorrectlyFromData)
{
  int n_points = 50;
  MatrixXd data(n_points,test_dim); // No phase

  for (int i=0; i<data.cols(); i++)
      data.col(i) = VectorXd::LinSpaced(n_points, 0.0, 1.0);

  EXPECT_NO_THROW(VirtualMechanismSpline<VMP_1ord_t>(data));
  EXPECT_NO_THROW(VirtualMechanismSpline<VMP_2ord_t>(data));

  data.resize(n_points,test_dim+1); // With phase

  for (int i=0; i<data.cols(); i++)
      data.col(i) = VectorXd::LinSpaced(n_points, 0.0, 1.0);

  EXPECT_NO_THROW(VirtualMechanismSpline<VMP_1ord_t>(data));
  EXPECT_NO_THROW(VirtualMechanismSpline<VMP_2ord_t>(data));
}*/

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
