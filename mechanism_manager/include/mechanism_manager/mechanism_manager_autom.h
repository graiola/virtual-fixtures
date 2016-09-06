/**
 * @file   mechanism_manager_autom.h
 * @brief  Autom for mechanism manager.
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

#ifndef MECHANISM_MANAGER_AUTOM_H
#define MECHANISM_MANAGER_AUTOM_H

#include "mechanism_manager/mechanism_manager.h"

namespace mechanism_manager
{

class MechanismManagerAutom
{
public:
    MechanismManagerAutom(const double phase_dot_preauto_th, const double phase_dot_th);
    void Step(const double phase_dot,const double phase_dot_ref, bool collision_detected);
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
