#ifndef AI_SERVER_UTIL_DETAIL_ALGORITHM_H
#define AI_SERVER_UTIL_DETAIL_ALGORITHM_H

#include <type_traits>

namespace ai_server {
namespace util {
namespace detail {

struct has_front_impl {
  template <class T>
  static auto check(T&& x) -> decltype(x.front(), std::true_type{});
  template <class T>
  static auto check(...) -> std::false_type;
};

/// @class   has_front
/// @brief   T がメンバ関数 front() を持っているか調べるメタ関数
template <class T>
class has_front : public decltype(has_front_impl::check<T>(std::declval<T>())) {};

struct has_top_impl {
  template <class T>
  static auto check(T&& x) -> decltype(x.top(), std::true_type{});
  template <class T>
  static auto check(...) -> std::false_type;
};

/// @class   has_top
/// @brief   T がメンバ関数 top() を持っているか調べるメタ関数
template <class T>
class has_top : public decltype(has_top_impl::check<T>(std::declval<T>())) {};

} // namespace detail
} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_DETAIL_ALGORITHM_H
