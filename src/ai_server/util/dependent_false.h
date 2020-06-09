#ifndef AI_SERVER_UTIL_DEPENDENT_FALSE_H
#define AI_SERVER_UTIL_DEPENDENT_FALSE_H

namespace ai_server::util {

// 型に依存するけど結果は常に false となる値
// constexpr if のエラー処理の workaround に使う
// http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2019/p1830r1.pdf
template <class>
inline constexpr bool dependent_false_v = false;

} // namespace ai_server::util

#endif // AI_SERVER_UTIL_DEPENDENT_FALSE_H
