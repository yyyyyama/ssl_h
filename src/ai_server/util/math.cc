#include <boost/math/constants/constants.hpp>
#include "ai_server/util/math.h"

template <class T>

T wrap_to_2pi(T r) {
  using boost::math::constants::two_pi;

  auto wrapped = std::fmod(r, two_pi<T>());

  if (r < 0) {
    wrapped += two_pi<T>();
  }
  return wrapped;
}
