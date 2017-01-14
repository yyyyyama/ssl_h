#ifndef AI_SERVER_UTIL_ALGORITHM_H
#define AI_SERVER_UTIL_ALGORITHM_H

#include <type_traits>

#include "detail/algorithm.h"

namespace ai_server {
namespace util {

/// @brief   キューやスタックの全ての要素に f を適用する
/// @param c キューやスタックのコンテナ
/// @param f 適用させたい関数
template <
    class Container, class Function,
    // Container がメンバ関数 front() と pop() を持っていたらこっちを呼び出す
    std::enable_if_t<detail::has_front_v<Container> && detail::has_pop_v<Container>>* = nullptr>
inline void pop_each(Container&& c, Function f) {
  while (!c.empty()) {
    decltype(auto) front = c.front();
    f(front);
    c.pop();
  }
}

/// @brief   キューやスタックの全ての要素に f を適用する
/// @param c キューやスタックのコンテナ
/// @param f 適用させたい関数
template <
    class Container, class Function,
    // Container がメンバ関数 top() と pop() を持っていたらこっちを呼び出す
    std::enable_if_t<detail::has_top_v<Container> && detail::has_pop_v<Container>>* = nullptr>
inline void pop_each(Container&& c, Function f) {
  while (!c.empty()) {
    decltype(auto) top = c.top();
    f(top);
    c.pop();
  }
}

} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_ALGORITHM_H
