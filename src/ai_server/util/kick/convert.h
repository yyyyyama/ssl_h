#ifndef AI_SERVER_UTIL_KICK_CONVERT_H
#define AI_SERVER_UTIL_KICK_CONVERT_H

#include <algorithm>

namespace ai_server::util::kick {

/// @brief       kick powerを速度(mm/s)に変換する
/// @param power 変換したいkick power
/// @return      0から8000の範囲で100*power
template <class T>
inline auto power_to_speed(T power) {
  return std::clamp(static_cast<double>(100 * power), 0.0, 8000.0);
}

/// @brief       速度(mm/s)をkick powerに変換する
/// @param speed 変換したい速度(mm/s)
/// @return　　　0から255の範囲でspeed/100
template <class T>
inline auto speed_to_power(T speed) {
  return std::clamp(static_cast<int>(speed / 100), 0, 255);
}

} // namespace ai_server::util::kick

#endif // AI_SERVER_UTIL_KICK_CONVERT_H
