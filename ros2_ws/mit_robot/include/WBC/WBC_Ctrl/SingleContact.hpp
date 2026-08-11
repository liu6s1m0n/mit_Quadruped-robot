// 旧目录兼容接口：单足点接触。新代码优先包含 WBC/ContactSet 下的头文件。
#ifndef Cheetah_SINGLE_CONTACT
#define Cheetah_SINGLE_CONTACT

#include <cstddef>

#include <WBC/FloatingBaseModel.h>
#include <WBC/ContactSpec.hpp>

template <typename T>
class SingleContact : public ContactSpec<T> {
 public:
  SingleContact(const FloatingBaseModel<T>* robot, int contact_pt);
  SingleContact(const FloatingBaseModel<T>& robot, std::size_t contact_pt);
  ~SingleContact() override = default;

  void setMaxFz(T max_fz) { _max_Fz = max_fz; }

 protected:
  T _max_Fz;
  std::size_t _contact_pt;
  std::size_t _dim_U;

  bool _UpdateJc() override;
  bool _UpdateJcDotQdot() override;
  bool _UpdateUf() override;
  bool _UpdateInequalityVector() override;

  const FloatingBaseModel<T>* robot_sys_;
};

#endif
