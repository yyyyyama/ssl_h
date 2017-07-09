#ifndef AI_SERVER_UTIL_DETAIL_TIME_H
#define AI_SERVER_UTIL_DETAIL_TIME_H

#include <chrono>
#include <type_traits>

namespace ai_server {
namespace util {
namespace detail {

// template型引数に与えた型がstd::chrono::durationかを判定するメタ関数
template <class>
struct is_duration : std::false_type {};

template <class Rep, class Period>
struct is_duration<std::chrono::duration<Rep, Period>> : std::true_type {};

template <class T>
static constexpr auto is_duration_v = is_duration<T>::value;

} // namespace detail
} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_DETAIL_TIME_H
