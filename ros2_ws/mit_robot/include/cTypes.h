/*! @file cTypes.h
 *  @brief C 和 C++ 通用的定宽整数类型别名
 *
 *  为 stdint.h 提供的无符号和有符号定宽整数定义简写别名。
 *  本文件保持 C 语法兼容，可同时用于 C 和 C++ 代码。
 */

#ifndef PROJECT_CTYPES_H
#define PROJECT_CTYPES_H

#include <stddef.h>  // 提供 size_t 等标准类型
#include <stdint.h>

// stdint.h 定宽整数类型的简写别名
typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

#endif  // PROJECT_CTYPES_H
