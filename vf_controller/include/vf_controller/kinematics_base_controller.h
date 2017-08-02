#ifndef KINEMATICS_BASE_CONTROLLER_H
#define KINEMATICS_BASE_CONTROLLER_H

////////// ROS
#include <ros/ros.h>
#include <controller_interface/controller.h>

////////// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

////////// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>

////////// BOOST
#include <boost/tokenizer.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace controller_interface {


template<typename JointInterface>
class KinematicsBaseController: public Controller<JointInterface>
{
public:

  KinematicsBaseController() {}
  ~KinematicsBaseController() {}

  bool init(JointInterface *robot, ros::NodeHandle &n);
  bool getHandles(JointInterface *robot);

protected:

  std::vector<typename JointInterface::ResourceHandleType> joint_handles_;
  ros::NodeHandle nh_;
  KDL::Chain kdl_chain_;
  KDL::Vector gravity_;

  boost::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
  boost::shared_ptr<KDL::ChainDynParam> id_solver_;
  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
};

}

#endif



