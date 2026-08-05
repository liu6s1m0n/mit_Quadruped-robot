#include "model/quadruped.hpp"

#include <stdexcept>

#include "model/robots/unitree_go1.hpp"

template<typename T>
Quadruped<T> makeQuadruped(RobotType robot_type)
{
  switch (robot_type) {
    case RobotType::UNITREE_GO1:
      return robots::unitree_go1::makeModel<T>();

    case RobotType::DM_BOT1:
      // DM_BOT1 已登记型号，但其独立参数文件尚未实现。
      throw std::invalid_argument("DM_BOT1 model parameters are not implemented yet");
  }

  throw std::invalid_argument("unsupported robot type");
}

// 通用容器和统一工厂当前只预编译 float 精度。
template class Quadruped<float>;
template Quadruped<float> makeQuadruped<float>(RobotType);
