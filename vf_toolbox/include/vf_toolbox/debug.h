/**
 * @file   debug.h
 * @brief  Debug utilities.
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

#ifndef DEBUG_H
#define DEBUG_H

////////// STD
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <string>
#include <sstream>

////////// ROS
#include <ros/ros.h>

struct stringwrapper
{
   std::stringstream ss;
   template<typename T>
   stringwrapper & operator << (const T &data)
   {
        ss << data;
        return *this;
   }
   operator std::string() { return ss.str(); }
};

#ifdef EIGEN_MALLOC_CHECKS
  #define EIGEN_RUNTIME_NO_MALLOC
  #define START_REAL_TIME_CRITICAL_CODE() do { Eigen::internal::set_is_malloc_allowed(false); } while (0) 
  #define END_REAL_TIME_CRITICAL_CODE() do { Eigen::internal::set_is_malloc_allowed(true); } while (0) 
#else
  #define START_REAL_TIME_CRITICAL_CODE() do {  } while (0) 
  #define END_REAL_TIME_CRITICAL_CODE() do {  } while (0) 
#endif

#define ROS_PRINTS
#ifdef ROS_PRINTS
     #define PRINT_INFO(f_) do { ROS_INFO_STREAM(f_); } while (0)
     #define PRINT_ERROR(f_) do { ROS_ERROR_STREAM(f_); throw std::runtime_error(stringwrapper() << f_); } while (0)
     #define PRINT_WARNING(f_) do { ROS_WARN_STREAM(f_); } while (0)
#else
    //#define PRINT_INFO(f_, ...) std::printf((std::string(f_)+"\n").c_str(), __VA_ARGS__);
    //#define PRINT_ERROR(f_, ...) std::printf(("ERROR: "+std::string(f_)+"\n").c_str(), __VA_ARGS__);
    #define PRINT_INFO(f_) do { std::cout << f_ << std::endl; } while (0)
    #define PRINT_ERROR(f_) do { std::cout << "ERROR: " << f_ << std::endl; throw std::runtime_error(stringwrapper() << f_); } while (0)
    #define PRINT_WARNING(f_) do { std::cout << "WARNING: " << f_ << std::endl; } while (0)
#endif


/// Here is some crazy code to map the number of arguments to a specific macro
/*
#define GET_MACRO(_0, _1, _2, _3, _4, NAME, ...) NAME
#define FOO(...) GET_MACRO(_0, ##__VA_ARGS__, FOO4, FOO3, FOO2, FOO1, FOO0)(__VA_ARGS__)
#define FOO0() do {} while (0)
#define FOO1(f) do { std::cout << std::string(f) << std::endl; } while (0)
#define FOO2(f,...) do { std::printf((std::string(f)+"\n").c_str(), __VA_ARGS__); } while (0)
#define FOO3(f,...) do { std::printf((std::string(f)+"\n").c_str(), __VA_ARGS__); } while (0)
#define FOO4(f,...) do { std::printf((std::string(f)+"\n").c_str(), __VA_ARGS__); } while (0)
*/

#endif
