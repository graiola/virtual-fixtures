#ifndef DEBUG_H
#define DEBUG_H

////////// STD
#include <iostream>
#include <fstream>

#ifdef EIGEN_MALLOC_CHECKS
  #define EIGEN_RUNTIME_NO_MALLOC
  #define START_REAL_TIME_CRITICAL_CODE() do { Eigen::internal::set_is_malloc_allowed(false); } while (0) 
  #define END_REAL_TIME_CRITICAL_CODE() do { Eigen::internal::set_is_malloc_allowed(true); } while (0) 
#else
  #define START_REAL_TIME_CRITICAL_CODE() do {  } while (0) 
  #define END_REAL_TIME_CRITICAL_CODE() do {  } while (0) 
#endif

//#define ROS_PRINTS
#ifdef ROS_PRINTS
     #define PRINT_INFO(f_) ROS_INFO_STREAM(f_);
     #define PRINT_ERROR(f_, ...) ROS_ERROR_STREAM(f_);
     #define PRINT_WARNING(f_, ...) ROS_WARNING_STREAM(f_);
#else
    //#define PRINT_INFO(f_, ...) std::printf((std::string(f_)+"\n").c_str(), __VA_ARGS__);
    //#define PRINT_ERROR(f_, ...) std::printf(("ERROR: "+std::string(f_)+"\n").c_str(), __VA_ARGS__);
    #define PRINT_INFO(f_) do { std::cout << f_ << std::endl; } while (0)
    #define PRINT_ERROR(f_) do { std::cout << "ERROR: " << f_ << std::endl; } while (0)
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
