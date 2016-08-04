#ifndef DEBUG_H
#define DEBUG_H

#ifdef EIGEN_MALLOC_CHECKS
  #define EIGEN_RUNTIME_NO_MALLOC
  #define START_REAL_TIME_CRITICAL_CODE() do { Eigen::internal::set_is_malloc_allowed(false); } while (0) 
  #define END_REAL_TIME_CRITICAL_CODE() do { Eigen::internal::set_is_malloc_allowed(true); } while (0) 
#else
  #define START_REAL_TIME_CRITICAL_CODE() do {  } while (0) 
  #define END_REAL_TIME_CRITICAL_CODE() do {  } while (0) 
#endif

#endif
