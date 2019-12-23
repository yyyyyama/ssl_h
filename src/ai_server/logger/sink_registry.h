#ifndef AI_SERVER_LOGGER_SINK_REGISTRY_H
#define AI_SERVER_LOGGER_SINK_REGISTRY_H

#include <mutex>
#include <set>

namespace ai_server::logger {
namespace sink {
class base;
}

struct log_item;

/// sink を管理する
class sink_registry final {
  mutable std::mutex mutex_;
  /// 登録されている sink
  std::set<sink::base*> sinks_;

  // sink_registry 内でのみ初期化できる
  sink_registry() = default;

public:
  // コピー・ムーブ不可
  sink_registry(const sink_registry&) = delete;
  sink_registry(sink_registry&&)      = delete;

  sink_registry& operator=(const sink_registry&) = delete;
  sink_registry& operator=(sink_registry&&) = delete;

  /// プロセス内で共通の sink_registry を取得する
  static sink_registry& global_sink_registry();

  /// @brief          sink を登録する
  /// @param sink     登録したい sink
  void register_sink(sink::base* sink);

  /// @brief          sink の登録を解除する
  /// @param sink     登録を解除したい sink
  void unregister_sink(sink::base* sink) noexcept;

  /// @brief          登録された sink が指す先を変更する
  /// @param from     移動元
  /// @param to       移動先
  void move(sink::base* from, sink::base* to) noexcept;

  /// @brief          登録された全て sink にログを出力する
  /// @param item     出力したいログ
  void notify_all(const log_item& item);
};

} // namespace ai_server::logger

#endif // AI_SERVER_LOGGER_SINK_REGISTRY_H
