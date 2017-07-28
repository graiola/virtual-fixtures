/**
 * @file   virtual_mechanism_factory.cpp
 * @brief  Factory to create easily the virtual mechanisms.
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

#include "virtual_mechanism/virtual_mechanism_factory.h"
#include "virtual_mechanism/virtual_mechanism_spline.h"
#include <vf_toolbox/vf_toolbox.h>

using namespace Eigen;
using namespace std;
using namespace tool_box;

namespace virtual_mechanism
{

typedef VirtualMechanismInterfaceFirstOrder VMP_1ord_t;
typedef VirtualMechanismInterfaceSecondOrder VMP_2ord_t;

VirtualMechanismFactory::VirtualMechanismFactory()
{
}

VirtualMechanismInterface* VirtualMechanismFactory::Build(const MatrixXd& data, const order_t order, const model_type_t model_type)
{
    VirtualMechanismInterface* vm_ptr = NULL;
    try
    {
        vm_ptr = CreateMechanism(data,order,model_type);
    }
    catch(const runtime_error& e)
    {
       PRINT_ERROR(e.what());
    }

    return vm_ptr;
}

VirtualMechanismInterface* VirtualMechanismFactory::Build(const string data_file, const order_t order, const model_type_t model_type)
{
    VirtualMechanismInterface* vm_ptr = NULL;
    Eigen::MatrixXd data;
    ReadTxtFile(data_file,data);
    vm_ptr = Build(data,order,model_type);
    return vm_ptr;
}

VirtualMechanismInterface* VirtualMechanismFactory::CreateMechanism(const MatrixXd& data, const order_t order, const model_type_t model_type)
{
     VirtualMechanismInterface* vm_ptr = NULL;

     switch(order)
     {
       case FIRST:
         vm_ptr = SelectModel<VMP_1ord_t>(data,model_type);
         break;
       case SECOND:
         vm_ptr = SelectModel<VMP_2ord_t>(data,model_type);
         break;
     }
     return vm_ptr;
}

template<typename ORDER>  VirtualMechanismInterface* VirtualMechanismFactory::SelectModel(const MatrixXd& data, const model_type_t model_type)
{
    VirtualMechanismInterface* vm_ptr = NULL;
    switch(model_type)
    {
       case SPLINE:
        vm_ptr = new VirtualMechanismSpline<ORDER>(data);
        break;
    }
    return vm_ptr;
}

} // namespace
