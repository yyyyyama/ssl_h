#ifndef AI_SERVER_UTIL_MATH_H
#define AI_SERVER_UTIL_MATH_H

#include <boost/math/constants/constants.hpp>
#include <cmath>

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
}
}
#endif
