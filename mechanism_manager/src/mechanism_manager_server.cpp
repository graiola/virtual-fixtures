/**
 * @file   mechanism_manager_server.cpp
 * @brief  ROS service server for mechanism manager.
 * @author Gennaro Raiola
 *
 * This file is part of virtual-fixtures, a set of libraries and programs to create
 * and interact with a library of virtual guides.
 * Copyright (C) 2014-2016 Gennaro Raiola, ENSTA-ParisTech
 *
 * virtual-fixtures is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * virtual-fixtures is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with virtual-fixtures.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "mechanism_manager/mechanism_manager_server.h"

using namespace mechanism_manager;
using namespace ros;

bool use_threading = false;

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
        mm_interface_->DeleteVm(req.selected_guide_idx,use_threading);
        res.response_command = req.request_command;
    }
    if(std::strcmp(req.request_command.c_str(), "save") == 0)
    {
        mm_interface_->SaveVm(req.selected_guide_idx,use_threading);
        res.response_command = req.request_command;
    }
    if(std::strcmp(req.request_command.c_str(), "insert") == 0)
    {
        mm_interface_->InsertVm(req.selected_guide_name,use_threading);
        res.response_command = req.request_command;
    }

    if(std::strcmp(req.request_command.c_str(), "set_name") == 0)
    {
        mm_interface_->SetVmName(req.selected_guide_idx,req.selected_guide_name);
        res.response_command = req.request_command;
    }

    if(std::strcmp(req.request_command.c_str(), "set_mode") == 0)
    {
        mm_interface_->SetVmMode(req.selected_mode);
        res.response_command = req.request_command;
    }

    if(std::strcmp(req.request_command.c_str(), "get_mode") == 0)
    {
        std::string selected_mode;
        mm_interface_->GetVmMode(selected_mode);
        res.selected_mode = selected_mode;
        res.response_command = req.request_command;
    }

    if(std::strcmp(req.request_command.c_str(), "set_merge_th") == 0)
    {
        mm_interface_->SetMergeThreshold(req.merge_th);
        res.response_command = req.request_command;
    }

    if(std::strcmp(req.request_command.c_str(), "get_merge_th") == 0)
    {
        double merge_th = 0;
        mm_interface_->GetMergeThreshold(merge_th);
        res.merge_th = merge_th;
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



