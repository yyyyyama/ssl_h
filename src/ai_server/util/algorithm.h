#ifndef AI_SERVER_UTIL_ALGORITHM_H
#define AI_SERVER_UTIL_ALGORITHM_H

#include <type_traits>

#include "detail/algorithm.h"

namespace ai_server {
namespace util {

/// @brief   キューやスタックの全ての要素に f を適用する
/// @param c キューやスタックのコンテナ
/// @param f 適用させたい関数
template <class Container, class Function,
          // Container がメンバ関数 front() を持っていたらこっちを呼び出す
          std::enable_if_t<detail::has_front<Container>::value>* = nullptr>
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
template <class Container, class Function,
          // Container がメンバ関数 top() を持っていたらこっちを呼び出す
          std::enable_if_t<detail::has_top<Container>::value>* = nullptr>
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
