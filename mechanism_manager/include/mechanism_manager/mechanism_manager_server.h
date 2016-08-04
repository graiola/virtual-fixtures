#ifndef MECHANISM_MANAGER_SERVER_H
#define MECHANISM_MANAGER_SERVER_H

//#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mechanism_manager/mechanism_manager.h>
#include <mechanism_manager/MechanismManagerAction.h>
#include <toolbox/toolbox.h>


namespace mechanism_manager{

class MechanismManagerServer
{


public:
    MechanismManagerServer(MechanismManager* mm, const ros::NodeHandle& nh);
    ~MechanismManagerServer();

    void Delete(const MechanismManagerGoalConstPtr &goal);

protected:
    actionlib::SimpleActionServer<mechanism_manager::MechanismManagerAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    mechanism_manager::MechanismManagerActionFeedback feedback_;
    mechanism_manager::MechanismManagerActionResult result_;

private:
    MechanismManager* mm_;
    ros::AsyncSpinner* spinner_ptr_; // Used to keep alive the ros callbacks
};

} // namespace

#endif // MECHANISM_MANAGER_INTERFACE_H
