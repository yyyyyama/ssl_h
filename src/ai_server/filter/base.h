#ifndef AI_SERVER_FILTER_BASE_H
#define AI_SERVER_FILTER_BASE_H

#include <chrono>
#include <functional>

// TODO: C++17移行時に<optional>にする
#include <experimental/optional>

#include "ai_server/util/time.h"

namespace ai_server {
namespace filter {

/// フィルタの状態を更新するタイミング
enum class timing {
  on_updated, // 対象データが更新されたとき
  manual,     // 任意のタイミング
};

template <class T, timing Timing>
class base;

template <class T>
/// @brief 新しい値を受け取ったときに更新されるfilter
class base<T, timing::on_updated> {
public:
  virtual ~base() = default;

  /// @brief                  filterの状態を更新する
  /// @param value            新しい値
  /// @param time             valueが取得された時刻
  /// @return                 filterを通した値
  ///
  /// filterの状態を更新するためのメンバ関数.
  /// updater::worldでは, Visionから新しい値を受け取ったときに呼び出される.
  virtual T update(const T& value, util::time_point_type time) = 0;
};

template <class T>
/// @brief 値の更新を任意のタイミングで行うfilter
class base<T, timing::manual> {
public:
  /// 最新の値を取得する関数オブジェクトの型
  using last_value_func_type = std::function<std::experimental::optional<T>(void)>;
  /// 対象の値を更新する関数オブジェクトの型
  using writer_func_type = std::function<void(std::experimental::optional<T>)>;

  /// @param last_value_func  最新の値を取得する関数オブジェクト
  /// @param writer_func      対象の値を更新する関数オブジェクト
  base(last_value_func_type last_value_func, writer_func_type writer_func)
      : last_value_func_(last_value_func), writer_func_(writer_func) {}

  virtual ~base() = default;

protected:
  /// @brief                  最新の値を取得する
  /// @return                 最新の値
  ///
  /// last_value_func_に設定された関数を使い, 対象の最新の値を取得する.
  /// 対象が存在しなかったときなどはstd::experimental::nulloptを返す.
  /// (updater::robotでは, ロボットがロストしていたときstd::experimental::nulloptを返す)
  std::experimental::optional<T> last_value() {
    if (last_value_func_) {
      return last_value_func_();
    } else {
      return std::experimental::nullopt;
    }
  }

  /// @brief                  対象の値を更新する
  /// @param value            更新する値
  ///
  /// writer_func_に設定された関数を使い, 対象の値をvalueで更新する.
  /// std::experimental::nulloptを渡した場合, 対象が存在しなかったとして扱われる.
  /// (updater::robotでは, ロボットがロストしていたとして処理される)
  void write(std::experimental::optional<T> value) {
    if (writer_func_) {
      writer_func_(std::move(value));
    }
  }

private:
  /// 最新の値を取得する関数オブジェクト
  last_value_func_type last_value_func_;
  /// 対象の値を更新する関数オブジェクト
  writer_func_type writer_func_;
};

} // namespace filter
} // namespace ai_server

#endif // AI_SERVER_FILTER_BASE_H
