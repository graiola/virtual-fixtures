#include <gtest/gtest.h>
#include "virtual_mechanism/virtual_mechanism_factory.h"

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

int n_points = 30;
int state_dim = 3;
VirtualMechanismFactory factory;

std::string pkg_path = ros::package::getPath("virtual_mechanism");
std::string file_path(pkg_path+"/test/test_virtual_mechanism_spline.model"); // TODO generalize

MatrixXd createData(int n_points, int state_dim)
{
  MatrixXd data(n_points,state_dim);
  for (int i=0; i<data.cols(); i++)
    data.col(i) = VectorXd::LinSpaced(n_points, 0.0, 1.0);
  return data;
}

TEST(VirtualMechanismFactory, buildFromData)
{
  MatrixXd data = createData(n_points,state_dim);
  EXPECT_NO_THROW(factory.Build(data,FIRST,SPLINE));
  EXPECT_NO_THROW(factory.Build(data,SECOND,SPLINE));
}

TEST(VirtualMechanismFactory, buildFromFile)
{
  EXPECT_NO_THROW(factory.Build(file_path,FIRST,SPLINE));
  EXPECT_NO_THROW(factory.Build(file_path,SECOND,SPLINE));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
