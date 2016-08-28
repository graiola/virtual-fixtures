/**
 * @file   mechanism_manager_server.h
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

private:
    MechanismManagerInterface* mm_interface_;
    ros::AsyncSpinner* spinner_ptr_; // Used to keep alive the ros callbacks
};

} // namespace

#endif // MECHANISM_MANAGER_INTERFACE_H
