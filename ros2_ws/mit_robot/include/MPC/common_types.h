/**
 * @file common_types.h
 * @brief 旧版凸 MPC C 接口使用的基础数值和整数类型别名。
 *
 * 新版 C++ MPC 主要使用 Eigen 和 RobotState/SolverMPC 类型；阅读主流程时
 * 本文件只需了解 fpt/mfp 以及定长整数别名即可。
 */
#ifndef _common_types
#define _common_types
#include <stdint.h>
#include <eigen3/Eigen/Dense>

//adding this line adds print statements and sanity checks
//that are too slow for realtime use.
//#define K_DEBUG

typedef double dbl;  ///< 旧版 double 浮点类型。
typedef float flt;   ///< 旧版 float 浮点类型。

//floating point type used whenever possible
typedef float fpt;   ///< 旧版主要计算浮点类型。

//floating point type used when interfacing with MATLAB
typedef double mfp;  ///< 与 MATLAB/旧接口交互时使用的 double 类型。
typedef int mint;    ///< 旧版整数类型。

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef int8_t   s8;
typedef int16_t  s16;
typedef int32_t  s32;
typedef int64_t  s64;

#endif
