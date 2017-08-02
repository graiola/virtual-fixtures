#include "vf_controller/kinematics_base_controller.h"
#include <hardware_interface/joint_command_interface.h>

using namespace std;
using namespace KDL;
using namespace ros;
using namespace Eigen;

namespace controller_interface {

template <typename JointInterface>
bool KinematicsBaseController<JointInterface>::init(JointInterface *robot, ros::NodeHandle &n)
{
    nh_ = n;

    // get URDF and name of root and tip from the parameter server
    std::string robot_description, root_name, tip_name;

    if (ros::param::search(nh_.getNamespace(),"robot_description", robot_description))
    {
        nh_.param(robot_description, robot_description, string());
    }
    else
    {
        ROS_ERROR_STREAM("KinematicsBaseController: No robot description (URDF) found on parameter server ("<<nh_.getNamespace()<<"/robot_description)");
        return false;
    }

    if (!nh_.getParam("root_name", root_name))
    {
        ROS_ERROR_STREAM("KinematicsBaseController: No root name found on parameter server ("<<nh_.getNamespace()<<"/root_name)");
        return false;
    }

    if (!nh_.getParam("tip_name", tip_name))
    {
        ROS_ERROR_STREAM("KinematicsBaseController: No tip name found on parameter server ("<<nh_.getNamespace()<<"/tip_name)");
        return false;
    }

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromString(robot_description, kdl_tree)){
      ROS_ERROR("KinematicsBaseController: Failed to construct kdl tree");
      n.shutdown();
      return false;
    }

    // Populate the KDL chain
    if(!kdl_tree.getChain(root_name, tip_name, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin(); it != segment_map.end(); it++ )
          ROS_ERROR_STREAM( "    "<<(*it).first);

        return false;
    }

    ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
    ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());

    // Get joint handles for all of the joints in the chain
    getHandles(robot);

    ROS_DEBUG("Number of joints in handle = %lu", joint_handles_.size());

    gravity_ = KDL::Vector::Zero();
    gravity_(2) = -9.81;

    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    id_solver_.reset(new KDL::ChainDynParam(kdl_chain_,gravity_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
}

template <typename JointInterface>
bool KinematicsBaseController<JointInterface>::getHandles(JointInterface *robot)
{
    for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
    {
        if ( it->getJoint().getType() != KDL::Joint::None )
        {
            joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
            ROS_DEBUG("%s", it->getJoint().getName().c_str() );
        }
    }
    return true;
}

template class KinematicsBaseController<hardware_interface::EffortJointInterface>;
}
