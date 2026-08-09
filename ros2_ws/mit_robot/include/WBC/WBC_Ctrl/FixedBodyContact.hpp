#ifndef Cheetah_FIXED_BODY_CONTACT
#define Cheetah_FIXED_BODY_CONTACT

#include <cstddef>

#include <WBC/FloatingBaseModel.h>
#include <WBC/ContactSpec.hpp>

template <typename T>
class FixedBodyContact : public ContactSpec<T> {
 public:
  /** Compatibility constructor for the project's 6 + 12 generalized velocities. */
  FixedBodyContact();
  explicit FixedBodyContact(std::size_t num_qdot);
  explicit FixedBodyContact(const FloatingBaseModel<T>* robot);
  explicit FixedBodyContact(const FloatingBaseModel<T>& robot);
  ~FixedBodyContact() override = default;

 protected:
  bool _UpdateJc() override;
  bool _UpdateJcDotQdot() override;
  bool _UpdateUf() override;
  bool _UpdateInequalityVector() override;
};

#endif
