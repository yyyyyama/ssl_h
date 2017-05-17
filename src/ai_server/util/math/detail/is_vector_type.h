#ifndef AI_SERVER_UTIL_MATH_DETAIL_IS_VECTOR_H
#define AI_SERVER_UTIL_MATH_DETAIL_IS_VECTOR_H

#include <type_traits>

namespace ai_server {
namespace util {
namespace math {
namespace detail {

/// @class   has_x_y
/// @brief   T がメンバ関数 x(), y() を持っているか調べるメタ関数
template <class T>
class has_x_y {
  template <class U>
  static auto check_x(U&& u) -> decltype(u.x(), std::true_type{});

  template <class U>
  static auto check_x(...) -> std::false_type;

  template <class U>
  static auto check_y(U&& u) -> decltype(u.y(), std::true_type{});

  template <class U>
  static auto check_y(...) -> std::false_type;

public:
  static constexpr bool value = decltype(check_x<T>(std::declval<T>()))::value &&
                                decltype(check_y<T>(std::declval<T>()))::value;
};

template <class T>
static constexpr bool has_x_y_v = has_x_y<T>::value;

/// @class   has_vx_vy
/// @brief   T がメンバ関数 vx(), vy() を持っているか調べるメタ関数
template <class T>
class has_vx_vy {
  template <class U>
  static auto check_vx(U&& u) -> decltype(u.vx(), std::true_type{});

  template <class U>
  static auto check_vx(...) -> std::false_type;

  template <class U>
  static auto check_vy(U&& u) -> decltype(u.vy(), std::true_type{});

  template <class U>
  static auto check_vy(...) -> std::false_type;

public:
  static constexpr bool value = decltype(check_vx<T>(std::declval<T>()))::value &&
                                decltype(check_vy<T>(std::declval<T>()))::value;
};

template <class T>
static constexpr bool has_vx_vy_v = has_vx_vy<T>::value;

/// @class   has_ax_ay
/// @brief   T がメンバ関数 ax(), ay() を持っているか調べるメタ関数
template <class T>
class has_ax_ay {
  template <class U>
  static auto check_vx(U&& u) -> decltype(u.vx(), std::true_type{});

  template <class U>
  static auto check_vx(...) -> std::false_type;

  template <class U>
  static auto check_vy(U&& u) -> decltype(u.vy(), std::true_type{});

  template <class U>
  static auto check_vy(...) -> std::false_type;

public:
  static constexpr bool value = decltype(check_vx<T>(std::declval<T>()))::value &&
                                decltype(check_vy<T>(std::declval<T>()))::value;
};

template <class T>
static constexpr bool has_ax_ay_v = has_ax_ay<T>::value;

} // namespace detail
} // namespace math
} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_MATH_DETAIL_IS_VECTOR_H
