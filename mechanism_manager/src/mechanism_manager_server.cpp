#include "mechanism_manager/mechanism_manager_server.h"

using namespace mechanism_manager;

MechanismManagerServer::MechanismManagerServer(MechanismManagerInterface* mm_interface, const ros::NodeHandle& nh)
    : as_(nh, nh.getNamespace(), boost::bind(&MechanismManagerServer::Delete, this, _1), false),
      action_name_(nh.getNamespace()),
      spinner_ptr_(NULL)
{
    assert(mm_interface!=NULL);

    mm_interface_ = mm_interface;

    if(ros::master::check())
    {
        as_.start();
        spinner_ptr_ = new ros::AsyncSpinner(1); // Use one thread to keep the ros magic alive
        spinner_ptr_->start();
    }
    else
    {
        std::string err("Can not start the action server, did you start roscore?");
        throw std::runtime_error(err);
    }
}

MechanismManagerServer::~MechanismManagerServer()
{
    if(spinner_ptr_!=NULL)
        delete spinner_ptr_;
    as_.shutdown();
}



void MechanismManagerServer::Delete(const MechanismManagerGoalConstPtr &goal)
{
   mm_interface_->DeleteVM(goal->delete_guide_idx);
   as_.setSucceeded();
}

/*void MechanismManagerInterface::Insert()
{
    mm_->InsertVM();
}*/



