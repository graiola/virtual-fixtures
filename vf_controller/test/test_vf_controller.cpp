#include <gtest/gtest.h>
#include "vf_controller/vf_controller.h"
#include <vf_toolbox/debug.h>


////////// STD
#include <iostream>
#include <fstream> 
#include <iterator>
#include <boost/concept_check.hpp>

using namespace vf_controller;
using namespace Eigen;
using namespace boost;

TEST(VFController, initializesCorrectly)
{
  // wait for ROS
  //bool isControllerAlive(){ return (odom_sub.getNumPublishers() > 0) && (cmd_pub.getNumSubscribers() > 0); }
  // while(!isControllerAlive())
   //{
   //  ros::Duration(0.1).sleep();
 //}
}

/*TEST(VFController, computeForwardKinematics)
{
  KinematicsBaseController kk(root_name,end_effector_name);
  VectorXd joints_pos(kk.getNdof());
  VectorXd qdot(kk.getNdof());
  VectorXd pose(kk.getCartSize());
  VectorXd velocity(kk.getCartSize());
  Vector3d position;
  Matrix3d orientation;
  
  
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(kk.ComputeFk(joints_pos,pose));
  EXPECT_NO_THROW(kk.ComputeFk(joints_pos,position,orientation));
  EXPECT_NO_THROW(kk.ComputeFkDot(joints_pos,qdot,velocity)); // NOTE: Rt problems with the interface
  END_REAL_TIME_CRITICAL_CODE();
}

TEST(KinematicsBaseController, computeJacobian)
{
  KinematicsBaseController kk(root_name,end_effector_name);

  VectorXd joints_pos(kk.getNdof());
  MatrixXd jacobian(kk.getCartSize(),kk.getNdof());

  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(kk.ComputeJac(joints_pos,jacobian));
  END_REAL_TIME_CRITICAL_CODE();
}

TEST(KinematicsBaseController, computeInverseKinematics)
{
  void ComputeIk(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, const Eigen::Ref<const Eigen::VectorXd>& v_in, Eigen::Ref<Eigen::VectorXd> qdot_out);
  
  KinematicsBaseController kk(root_name,end_effector_name);

  VectorXd joints_pos(kk.getNdof());
  VectorXd velocity(kk.getCartSize());
  VectorXd qdot(kk.getNdof());
  
  START_REAL_TIME_CRITICAL_CODE();
  EXPECT_NO_THROW(kk.ComputeIk(joints_pos,velocity,qdot));
  END_REAL_TIME_CRITICAL_CODE();
}*/

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_vf_controller");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
