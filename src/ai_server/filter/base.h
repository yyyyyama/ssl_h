#ifndef AI_SERVER_FILTER_BASE_H
#define AI_SERVER_FILTER_BASE_H

#include <chrono>
#include <functional>
#include <mutex>
#include <optional>

namespace ai_server {
namespace filter {

/// フィルタの状態を更新するタイミング
enum class timing {
  same,   // 対象データが更新されたとき
  manual, // 任意のタイミング
};

template <class T, timing Timing>
class base;

template <class T>
/// @brief 新しい値を受け取ったときに更新されるfilter
class base<T, timing::same> {
public:
  virtual ~base() = default;

  /// @brief                  filterの状態を更新する
  /// @param value            新しい値
  /// @param time             valueが取得された時刻
  /// @return                 filterを通した値
  ///
  /// filterの状態を更新するためのメンバ関数.
  /// updater::worldでは, Visionから新しい値を受け取ったときに呼び出される.
  /// 対象がロストしたときは引数にnulloptが渡される
  virtual std::optional<T> update(std::optional<T> value,
                                  std::chrono::system_clock::time_point time) = 0;
};

template <class T>
/// @brief 値の更新を任意のタイミングで行うfilter
class base<T, timing::manual> {
public:
  /// 対象の値を更新する関数オブジェクトの型
  using writer_func_type = std::function<void(std::optional<T>)>;

  /// @param writer_func      対象の値を更新する関数オブジェクト
  base(std::recursive_mutex& mutex, writer_func_type writer_func)
      : mutex_{mutex}, writer_func_(writer_func) {}

  virtual ~base() = default;

  virtual void set_raw_value(std::optional<T> value,
                             std::chrono::system_clock::time_point time) = 0;

protected:
  /// @brief                  対象の値を更新する
  /// @param value            更新する値
  ///
  /// writer_func_に設定された関数を使い, 対象の値をvalueで更新する.
  /// std::nulloptを渡した場合, 対象が存在しなかったとして扱われる.
  /// (updater::robotでは, ロボットがロストしていたとして処理される)
  void write(std::optional<T> value) {
    if (writer_func_) {
      writer_func_(std::move(value));
    }
  }

  std::recursive_mutex& mutex() {
    return mutex_;
  }

private:
  std::recursive_mutex& mutex_;

  /// 対象の値を更新する関数オブジェクト
  writer_func_type writer_func_;
};

} // namespace filter
} // namespace ai_server

#endif // AI_SERVER_FILTER_BASE_H
