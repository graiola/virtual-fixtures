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

using namespace virtual_mechanism;
using namespace Eigen;
using namespace boost;

typedef VirtualMechanismInterfaceFirstOrder VMP_1ord_t;
typedef VirtualMechanismInterfaceSecondOrder VMP_2ord_t;

int n_points = 30;
int state_dim = 3;
double dt = 0.001;

MatrixXd createData(int n_points, int state_dim)
{
  MatrixXd data(n_points,state_dim);
  for (int i=0; i<data.cols(); i++)
    data.col(i) = VectorXd::LinSpaced(n_points, 0.0, 1.0);
  return data;
}

TEST(VirtualMechanismSpline, initializesCorrectly)
{
  MatrixXd data = createData(n_points,state_dim);
  EXPECT_NO_THROW(VirtualMechanismSpline<VMP_1ord_t> vm1(data));
  EXPECT_NO_THROW(VirtualMechanismSpline<VMP_2ord_t> vm2(data));
}

TEST(VirtualMechanismSpline, update)
{
  MatrixXd data = createData(n_points,state_dim);
  VirtualMechanismSpline<VMP_1ord_t> vm1(data);
  VirtualMechanismSpline<VMP_2ord_t> vm2(data);
  VectorXd robot_pos(state_dim);
  VectorXd robot_vel(state_dim);
  EXPECT_NO_THROW(vm1.Update(robot_pos,robot_vel,dt));
  EXPECT_NO_THROW(vm2.Update(robot_pos,robot_vel,dt));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
