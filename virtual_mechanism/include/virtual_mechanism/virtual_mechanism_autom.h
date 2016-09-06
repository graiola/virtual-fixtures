/**
 * @file   virtual_mechanism_autom.h
 * @brief  Autom for virtual mechanism.
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

#ifndef VIRTUAL_MECHANISM_AUTOM_H
#define VIRTUAL_MECHANISM_AUTOM_H

////////// Toolbox
#include <toolbox/toolbox.h>

namespace virtual_mechanism
{

class VirtualMechanismAutom
{
public:
    VirtualMechanismAutom(){}
    VirtualMechanismAutom(const double phase_dot_preauto_th, const double phase_dot_th);
    void Step(const double& phase_dot, const double& phase_dot_ref, bool& collision_detected);
    bool GetState();
private:
    enum state_t {MANUAL,PREAUTO,AUTO};
    double phase_dot_preauto_th_;
    double phase_dot_th_;
    state_t state_;
    long long loopCnt;
};

} // namespace

#endif
