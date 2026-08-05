/*! @file cppTypes.h
 *  @brief C++ 专用的通用数学类型
 *
 *  定义基于 Eigen 的定长、动态矩阵/向量模板别名，
 *  以及满足 Eigen 内存对齐要求的 std::vector 别名。
 *  本文件使用 C++ 模板和 using 声明，不能被 C 代码包含。
 */

#ifndef PROJECT_CPPTYPES_H
#define PROJECT_CPPTYPES_H

#include <vector>
#include "cTypes.h"
#include <eigen3/Eigen/Dense>

// 3x3 旋转矩阵
template <typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;

// 2x1 列向量
template <typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// 3x1 列向量
template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

// 4x1 列向量
template <typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

// 6x1 列向量
template <typename T>
using Vec6 = Eigen::Matrix<T, 6, 1>;

// 10x1 列向量
template <typename T>
using Vec10 = Eigen::Matrix<T, 10, 1>;

// 12x1 列向量
template <typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;

// 18x1 列向量
template <typename T>
using Vec18 = Eigen::Matrix<T, 18, 1>;

// 28x1 列向量
template <typename T>
using Vec28 = Eigen::Matrix<T, 28, 1>;

// 3x3 矩阵
template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

// 4x1 四元数系数列向量
template <typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

// 6x1 空间向量
template <typename T>
using SVec = typename Eigen::Matrix<T, 6, 1>;

// 6x6 空间变换矩阵
template <typename T>
using SXform = typename Eigen::Matrix<T, 6, 6>;

// 6x6 矩阵
template <typename T>
using Mat6 = typename Eigen::Matrix<T, 6, 6>;

// 12x12 矩阵
template <typename T>
using Mat12 = typename Eigen::Matrix<T, 12, 12>;

// 18x18 矩阵
template <typename T>
using Mat18 = Eigen::Matrix<T, 18, 18>;

// 28x28 矩阵
template <typename T>
using Mat28 = Eigen::Matrix<T, 28, 28>;

// 3x4 矩阵
template <typename T>
using Mat34 = Eigen::Matrix<T, 3, 4>;

// 2x3 矩阵
template <typename T>
using Mat23 = Eigen::Matrix<T, 2, 3>;

// 4x4 矩阵
template <typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;

// 10x1 质量属性列向量
template <typename T>
using MassProperties = typename Eigen::Matrix<T, 10, 1>;

// 动态长度列向量
template <typename T>
using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

// 动态行列矩阵
template <typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// 由 6x1 空间向量组成的动态列数矩阵
template <typename T>
using D6Mat = typename Eigen::Matrix<T, 6, Eigen::Dynamic>;

// 由 3x1 笛卡尔向量组成的动态列数矩阵
template <typename T>
using D3Mat = typename Eigen::Matrix<T, 3, Eigen::Dynamic>;

// 使用 Eigen 对齐分配器的 std::vector
template <typename T>
using vectorAligned = typename std::vector<T, Eigen::aligned_allocator<T>>;

// 当前声明支持的 MIT Cheetah 机器人型号
enum class RobotType { CHEETAH_3, MINI_CHEETAH };

#endif  // PROJECT_CPPTYPES_H
