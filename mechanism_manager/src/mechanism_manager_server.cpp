#include "mechanism_manager/mechanism_manager_server.h"

using namespace mechanism_manager;
using namespace ros;

MechanismManagerServer::MechanismManagerServer(MechanismManagerInterface* mm_interface, NodeHandle& nh)
    : spinner_ptr_(NULL)
{
    assert(mm_interface!=NULL);

    mm_interface_ = mm_interface;

    if(master::check())
    {
        ss_ = nh.advertiseService("mechanism_manager_interaction",
                                  &MechanismManagerServer::CallBack, this);

        spinner_ptr_ = new AsyncSpinner(1); // Use one thread to keep the ros magic alive
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
}

bool MechanismManagerServer::CallBack(MechanismManagerServices::Request &req,
                                      MechanismManagerServices::Response &res)
{
    if(std::strcmp(req.request_command.c_str(), "delete") == 0)
    {
        mm_interface_->DeleteVm(req.selected_guide_idx);
        res.response_command = req.request_command;
    }
    if(std::strcmp(req.request_command.c_str(), "save") == 0)
    {
        mm_interface_->SaveVm(req.selected_guide_idx);
        res.response_command = req.request_command;
    }
    if(std::strcmp(req.request_command.c_str(), "insert") == 0)
    {
        mm_interface_->InsertVm(req.selected_guide_name);
        res.response_command = req.request_command;
    }

    if(std::strcmp(req.request_command.c_str(), "set_name") == 0)
    {
        mm_interface_->SetVmName(req.selected_guide_idx,req.selected_guide_name);
        res.response_command = req.request_command;
    }

    // Update the names list
    mm_interface_->GetVmNames(res.list_guides);


    return true;
}

/*void MechanismManagerInterface::Insert()
{
    mm_->InsertVm();
}*/



