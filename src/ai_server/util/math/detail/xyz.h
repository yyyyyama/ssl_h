#ifndef AI_SERVER_UTIL_MATH_DETAIL_XYZ_H
#define AI_SERVER_UTIL_MATH_DETAIL_XYZ_H

#include <tuple>
#include <type_traits>

namespace ai_server::util::math::detail {

template <class T>
inline auto x(const T& v) -> decltype(v.x) {
  return v.x;
}

template <class T>
inline auto x(const T& v) -> decltype(v.x()) {
  return v.x();
}

template <class... TArgs, std::enable_if_t<sizeof...(TArgs) >= 1 && sizeof...(TArgs) <= 3,
                                           std::nullptr_t> = nullptr>
inline auto x(const std::tuple<TArgs...>& v) {
  return std::get<0>(v);
}

template <class T>
inline auto y(const T& v) -> decltype(v.y) {
  return v.y;
}

template <class T>
inline auto y(const T& v) -> decltype(v.y()) {
  return v.y();
}

template <class... TArgs, std::enable_if_t<sizeof...(TArgs) >= 2 && sizeof...(TArgs) <= 3,
                                           std::nullptr_t> = nullptr>
inline auto y(const std::tuple<TArgs...>& v) {
  return std::get<1>(v);
}

template <class T>
inline auto z(const T& v) -> decltype(v.z) {
  return v.z;
}

template <class T>
inline auto z(const T& v) -> decltype(v.z()) {
  return v.z();
}

template <class... TArgs, std::enable_if_t<sizeof...(TArgs) == 3, std::nullptr_t> = nullptr>
inline auto z(const std::tuple<TArgs...>& v) {
  return std::get<2>(v);
}

} // namespace ai_server::util::math::detail

#endif
