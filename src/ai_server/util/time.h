#ifndef AI_SERVER_UTIL_ALGORITHM_H
#define AI_SERVER_UTIL_ALGORITHM_H

#include "detail/time.h"

namespace ai_server {
namespace util {

/// プロジェクト内で用いるclock
using clock_type = std::chrono::high_resolution_clock;

/// プロジェクト内で用いる時間を表現する型
using duration_type = typename clock_type::duration;

/// プロジェクト内で用いる時刻を表現する型
using type_point_type = typename clock_type::time_point;

/// @brief        浮動小数点数で表現された時間[s]をstd::chrono::durationに変換する
/// @param time   変換する時間
/// @return       変換された時間
///
/// VisionやRefBoxがこのような形式で時間を送ってくるので, それを変換するために実装.
/// Durationを明示的に指定しなかった場合, microsecオーダでの変換を行う.
template <class Duration = std::chrono::microseconds,
          std::enable_if_t<detail::is_duration_v<Duration>, std::nullptr_t> = nullptr>
inline constexpr Duration to_duration(double time) {
  using rep    = typename Duration::rep;
  using period = typename Duration::period;
  return Duration{static_cast<rep>((time * period::den) / period::num)};
}

} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_ALGORITHM_H
