/**
 * @file   virtual_mechanism_factory.h
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

#ifndef VIRTUAL_MECHANISM_FACTORY_H
#define VIRTUAL_MECHANISM_FACTORY_H

#include "virtual_mechanism/virtual_mechanism_interface.h"

namespace virtual_mechanism
{

enum order_t {FIRST,SECOND};
enum model_type_t {GMR,GMR_NORMALIZED}; // SPLINE

/*class VirtualMechanismAbstractFactory
{
public:
    virtual VirtualMechanismInterface* Build(const Eigen::MatrixXd& data, const order_t order, const model_type_t model_type) = 0;
    virtual VirtualMechanismInterface* Build(const std::string model_name, const order_t order, const model_type_t model_type) = 0;
};*/

class VirtualMechanismFactory// : public VirtualMechanismAbstractFactory
{
public:
    VirtualMechanismFactory();
    VirtualMechanismInterface* Build(const Eigen::MatrixXd& data, const order_t order, const model_type_t model_type);
    VirtualMechanismInterface* Build(const std::string model_name, const order_t order, const model_type_t model_type);
    VirtualMechanismInterface* Build(const Eigen::MatrixXd& data); // With default order and model_type
    VirtualMechanismInterface* Build(const std::string model_name); // With default order and model_type
    void SetDefaultPreferences(const order_t order, const model_type_t model_type);
    void SetDefaultPreferences(const std::string order, const std::string model_type);
protected:
    VirtualMechanismInterface* CreateEmptyMechanism(const order_t order, const model_type_t model_type);
    template<typename ORDER>  VirtualMechanismInterface* SelectModel(const model_type_t model_type);
    order_t default_order_;
    model_type_t default_model_type_;
};

} // namespace

#endif
