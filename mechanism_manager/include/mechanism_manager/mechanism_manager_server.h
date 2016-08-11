#ifndef MECHANISM_MANAGER_SERVER_H
#define MECHANISM_MANAGER_SERVER_H

#include <mechanism_manager/mechanism_manager_interface.h>
#include <mechanism_manager/MechanismManagerServices.h>
#include <toolbox/toolbox.h>

namespace mechanism_manager{

class MechanismManagerServer
{

public:
    MechanismManagerServer(MechanismManagerInterface* mm_interface, ros::NodeHandle& nh);
    ~MechanismManagerServer();

    bool CallBack(mechanism_manager::MechanismManagerServices::Request &req,
                  mechanism_manager::MechanismManagerServices::Response &res);

protected:
    ros::ServiceServer ss_;
    /*actionlib::SimpleActionServer<mechanism_manager::MechanismManagerAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    mechanism_manager::MechanismManagerActionFeedback feedback_;
    mechanism_manager::MechanismManagerActionResult result_;*/

private:
    MechanismManagerInterface* mm_interface_;
    ros::AsyncSpinner* spinner_ptr_; // Used to keep alive the ros callbacks
};

} // namespace

#endif // MECHANISM_MANAGER_INTERFACE_H
