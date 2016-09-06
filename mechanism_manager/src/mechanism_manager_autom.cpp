/**
 * @file   mechanism_manager_autom.cpp
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

#include "mechanism_manager/mechanism_manager_autom.h"

using namespace mechanism_manager;

MechanismManagerAutom::MechanismManagerAutom(const double phase_dot_preauto_th, const double phase_dot_th)
{
    assert(phase_dot_th > 0.0);
    assert(phase_dot_preauto_th > phase_dot_th);
    phase_dot_preauto_th_ = phase_dot_preauto_th;
    phase_dot_th_ = phase_dot_th;
    state_ = MANUAL;
    loopCnt = 0;
}

void MechanismManagerAutom::Step(const double phase_dot,const double phase_dot_ref, bool collision_detected)
{
    if(true)
    {
        switch(state_)
        {
            case MANUAL:
                if(phase_dot >= phase_dot_preauto_th_)
                    state_ = PREAUTO;
                break;
            case PREAUTO:
                if(phase_dot <= (phase_dot_ref + phase_dot_th_))
                    state_ = AUTO;
                break;
            case AUTO:
                //if((phase_dot < (phase_dot_ref - phase_dot_th_)))
                //if((std::abs(r) > (r_th_)))
                if(collision_detected)
                    state_ = MANUAL;
                break;
        }
    }
    else // Two states version
    {
            if((phase_dot <= (phase_dot_ref + phase_dot_th_)) && (phase_dot >= (phase_dot_ref - phase_dot_th_)))
                state_ = AUTO;
            else
                state_ = MANUAL;
    }
}

bool MechanismManagerAutom::GetState()
{
    bool activate_vm;
    switch(state_)
    {
        case MANUAL:
            activate_vm = false;
            break;
        case PREAUTO:
            activate_vm = false;
            break;
        case AUTO:
            activate_vm = true;
            break;
    }

    if(loopCnt%1000==0)
    {
        switch(state_)
        {
            case MANUAL:
                std::cout << "****" <<std::endl;
                std::cout << "MANUAL" <<std::endl;
                break;
            case PREAUTO:
                std::cout << "****" <<std::endl;
                std::cout << "PREAUTO" <<std::endl;
                break;
            case AUTO:
                std::cout << "****" <<std::endl;
                std::cout << "AUTO" <<std::endl;
                break;
        }
    }
    loopCnt++;

    return activate_vm;
}
