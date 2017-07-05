#ifndef AI_SERVER_UTIL_MATH_H
#define AI_SERVER_UTIL_MATH_H

#include <boost/math/constants/constants.hpp>

namespace ai_server {
namespace util {
template <class T>
T wrap_to_2pi(T r) {
  using boost::math::constants::two_pi;

  auto wrapped = std::fmod(r, two_pi<T>());

  if (r < 0) {
    wrapped += two_pi<T>();
  }
  return wrapped;
}

/// @brief	-pi<r<=piに正規化
template <class T>
T wrap_to_pi(T r) {
  using boost::math::constants::two_pi;
  using boost::math::constants::pi;

  auto wrapped = std::fmod(r, two_pi<T>());

  if (wrapped > pi<T>()) {
    wrapped -= two_pi<T>();
  } else if (wrapped <= -pi<T>()) {
    wrapped += two_pi<T>();
  }
  return wrapped;
}
}
}
#endif
