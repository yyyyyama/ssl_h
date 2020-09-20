#ifndef AI_SERVER_LOGGER_LOGGER_H
#define AI_SERVER_LOGGER_LOGGER_H

#include <chrono>
#include <string>
#include <thread>

#include <boost/type_index.hpp>

#include "ai_server/logger/log_item.h"
#include "ai_server/logger/sink_registry.h"

namespace ai_server::logger {

/// 登録された sink に対してログを出力する
class logger {
  std::string zone_name_;

public:
  /// @param zone_name     この logger の zone 名
  logger(std::string zone_name);

  // 各 log_level に対する log(...) へのエイリアス
  template <class String>
  inline void error(String&& msg) const {
    log(log_level::error, std::forward<String>(msg));
  }

  template <class String>
  inline void warn(String&& msg) const {
    log(log_level::warn, std::forward<String>(msg));
  }

  template <class String>
  inline void info(String&& msg) const {
    log(log_level::info, std::forward<String>(msg));
  }

  template <class String>
  inline void debug([[maybe_unused]] String&& msg) const {
#ifdef AI_SERVER_DEBUG
    log(log_level::debug, std::forward<String>(msg));
#endif
  }

  template <class String>
  inline void trace([[maybe_unused]] String&& msg) const {
#ifdef AI_SERVER_DEBUG
    log(log_level::trace, std::forward<String>(msg));
#endif
  }

private:
  /// @brief          ログを出力する
  /// @param level    ログの重要度
  /// @param msg      ログメッセージ
  inline void log(log_level level, std::string msg) const {
    log_item item{};
    item.level      = level;
    item.zone_name  = zone_name_;
    item.message    = std::move(msg);
    item.time_stamp = std::chrono::steady_clock::now();
    item.thread_id =
        static_cast<std::size_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));

    sink_registry::global_sink_registry().notify_all(item);
  }
};

template <class T>
/// zone_name が T の型名で初期化される logger
class logger_for : public logger {
public:
  logger_for() : logger{boost::typeindex::type_id<T>().pretty_name()} {}
};

} // namespace ai_server::logger

#endif // AI_SERVER_LOGGER_LOGGER_H
