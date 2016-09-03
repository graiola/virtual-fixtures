/**
 * @file   test_mechanism_manager.h
 * @brief  Create a real time loop and measure the computation time of mechanism manager update.
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

#include <toolbox/debug.h>
#include "mechanism_manager/mechanism_manager_interface.h"

////////// RTAI
extern "C"
{
#include <rtai_lxrt.h>
#include <rtai_nam2num.h>
}

////////// ROS
#include <ros/ros.h>

////////// SOME RT MACROS AND DEFINES
#define NANO2SEC(a)	a/1e9
#define SEC2NANO(a)	a*1e9

////////// Activate some timing infos
//#define TIMING // NOTE: Prints are not real time safe... use them at your own risk!
bool kill_loop = false;
static int tmp_dt_cnt;
static int tmp_loop_cnt;
static long long start_dt_time, end_dt_time, elapsed_dt_time;
static long long start_loop_time, end_loop_time, elapsed_loop_time;

#ifdef TIMING
#define TIME_ACTIVE 1
#else
#define TIME_ACTIVE 0
#endif

void getCpuCount(long long& out){
    out = nano2count(rt_get_cpu_time_ns());
}

double count2Sec(const long long in){
    return (NANO2SEC((double)count2nano(in)));
}

#define INIT_CNT(cnt) do { if (TIME_ACTIVE) (cnt) = 0; } while (0)
#define SAVE_TIME(out) do { if (TIME_ACTIVE) getCpuCount((out)); } while (0)
#define PRINT_TIME(T_start,T_end,cnt,string) do { if (TIME_ACTIVE) if ((cnt)%1000==0) ROS_INFO("%s: %fs",string,count2Sec(((T_end) - (T_start)))); cnt = cnt++ & INT_MAX;} while (0)

double dt = 0.001;
static RT_TASK *rt_task; // Main task will be Real time schedulable!

// NOTE: The rtai modules have to be already loaded in order to be able to schedule a rtai task
// sudo insmod /usr/realtime/modules/rtai_hal.ko
// sudo insmod /usr/realtime/modules/rtai_sched.ko
// NOTE: The code has to be compiled in Release in order to run at ~1kHz
bool rt_init()
{
    rt_allow_nonroot_hrt();
    //Args: Name, Priority, Stack Size, max_msg_size, Policy, cpus_allowed
    if (!(rt_task = rt_task_init_schmod(nam2num( "TestRT" ),0,0,0,SCHED_FIFO,0)))
    {
        ROS_ERROR("Cannot initialize the real time task");
        return false;
    }
    // Set the task period
    RTIME tick_period = nano2count(SEC2NANO(dt));

    if (!rt_is_hard_timer_running()) // Check if the timer is started
    {
        ROS_INFO("Starting Real Time Timer...\n");
        start_rt_timer(0);
    }

    rt_task_make_periodic(rt_task, rt_get_time() + tick_period, tick_period);

    mlockall(MCL_CURRENT | MCL_FUTURE); // Prevent memory swaps
    //rt_grow_and_lock_stack(1024);

    rt_make_hard_real_time(); // Make the task real time, ready to be scheduled by RTAI

    return true;
}

void rt_update_loop()
{
    INIT_CNT(tmp_dt_cnt);
    INIT_CNT(tmp_loop_cnt);
    if(rt_init())
    {
        mechanism_manager::MechanismManagerInterface mm;
        int pos_dim = mm.GetPositionDim();
        Eigen::VectorXd rob_pos(pos_dim);
        Eigen::VectorXd rob_vel(pos_dim);
        Eigen::VectorXd f_out(pos_dim);

        rob_pos.fill(0.25);
        rob_vel.fill(1.0);
        f_out.fill(0.0);

        SAVE_TIME(start_loop_time);
        while(!kill_loop) // RT Loop
        {
            SAVE_TIME(start_dt_time);

            START_REAL_TIME_CRITICAL_CODE();
            mm.Update(rob_pos,rob_vel,dt,f_out);
            END_REAL_TIME_CRITICAL_CODE();

            rt_task_wait_period(); // Wait until the end of the period.
            SAVE_TIME(end_dt_time);
            PRINT_TIME(start_dt_time,end_dt_time,tmp_dt_cnt,"dt");
        }
        SAVE_TIME(end_loop_time);
        PRINT_TIME(start_loop_time,end_loop_time,tmp_loop_cnt,"elapsed time");
        rt_task_delete(rt_task);
    }
}

void rt_shutdown(int signum)
{
    kill_loop = true;
}

int main(int argc, char** argv)
{
    signal(SIGINT, rt_shutdown);

    rt_update_loop();

    return EXIT_SUCCESS;
}
