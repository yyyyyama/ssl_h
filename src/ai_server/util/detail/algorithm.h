#ifndef AI_SERVER_UTIL_DETAIL_ALGORITHM_H
#define AI_SERVER_UTIL_DETAIL_ALGORITHM_H

#include <type_traits>

namespace ai_server {
namespace util {
namespace detail {

/// @class   has_front
/// @brief   T がメンバ関数 front() を持っているか調べるメタ関数
template <class T>
class has_front {
  template <class U>
  static auto check(U&& x) -> decltype(x.front(), std::true_type{});

  template <class U>
  static auto check(...) -> std::false_type;

public:
  static constexpr bool value = decltype(check<T>(std::declval<T>()))::value;
};

template <class T>
static constexpr bool has_front_v = has_front<T>::value;

/// @class   has_pop
/// @brief   T がメンバ関数 pop() を持っているか調べるメタ関数
template <class T>
class has_pop {
  template <class U>
  static auto check(U&& x) -> decltype(x.pop(), std::true_type{});

  template <class U>
  static auto check(...) -> std::false_type;

public:
  static constexpr bool value = decltype(check<T>(std::declval<T>()))::value;
};

template <class T>
static constexpr bool has_pop_v = has_pop<T>::value;

/// @class   has_top
/// @brief   T がメンバ関数 top() を持っているか調べるメタ関数
template <class T>
class has_top {
  template <class U>
  static auto check(U&& x) -> decltype(x.top(), std::true_type{});

  template <class U>
  static auto check(...) -> std::false_type;

public:
  static constexpr bool value = decltype(check<T>(std::declval<T>()))::value;
};

template <class T>
static constexpr bool has_top_v = has_top<T>::value;

} // namespace detail
} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_DETAIL_ALGORITHM_H
