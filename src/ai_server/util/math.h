#ifndef AI_SERVER_UTIL_MATH_H
#define AI_SERVER_UTIL_MATH_H

#include "math/angle.h"

namespace ai_server {
namespace util {

template <class T>
[[deprecated("This function was moved. Use util::math::wrap_to_2pi instead.")]] auto
wrap_to_2pi(T r) {
  return math::wrap_to_2pi(r);
}

template <class T>
[[deprecated("This function was moved. Use util::math::wrap_to_pi instead.")]] auto wrap_to_pi(
    T r) {
  return math::wrap_to_pi(r);
}
} // namespace util
} // namespace ai_server
#endif
