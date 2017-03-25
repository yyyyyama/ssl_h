#ifndef AI_SERVER_FILTER_BASE_H
#define AI_SERVER_FILTER_BASE_H

#include <chrono>

namespace ai_server {
namespace filter {

template <class T>
class base {
public:
  virtual ~base() = default;

  /// @brief                  objectにFilterを適用する
  /// @param object           Filterを適用するオブジェクト
  /// @param time             objectが更新された時刻
  virtual void apply(T& object, std::chrono::high_resolution_clock::time_point time) = 0;

  /// @brief                  Filterの状態を初期化する
  virtual void reset() {}
};

} // namespace filter
} // namespace ai_server

#endif // AI_SERVER_FILTER_BASE_H
