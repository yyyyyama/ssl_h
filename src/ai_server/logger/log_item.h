#ifndef AI_SERVER_LOGGER_LOG_ITEM_H
#define AI_SERVER_LOGGER_LOG_ITEM_H

#include <chrono>
#include <iosfwd>
#include <string>
#include <string_view>
#include <thread>

namespace ai_server::logger {

/// ログの重要度 (level) を表現する型
enum class log_level {
  trace,
  debug,
  info,
  warn,
  error,
};

/// ログ1項目を表現する型
struct log_item {
  log_level level;
  std::string zone_name;
  std::string message;
  std::chrono::steady_clock::time_point time_stamp;
  std::size_t thread_id;
};

/// log_level を文字列に変換する
static constexpr std::string_view log_level_to_string(log_level level) {
  using namespace std::string_view_literals;
  switch (level) {
    case log_level::trace:
      return "trace"sv;
    case log_level::debug:
      return "debug"sv;
    case log_level::info:
      return "info"sv;
    case log_level::warn:
      return "warn"sv;
    case log_level::error:
      return "error"sv;
  }
}

/// log_level を記号に変換する
static constexpr std::string_view log_level_to_symbol(log_level level) {
  using namespace std::string_view_literals;
  switch (level) {
    case log_level::trace:
      return "[TRACE]"sv;
    case log_level::debug:
      return "[DEBUG]"sv;
    case log_level::info:
      return "[+]"sv;
    case log_level::warn:
      return "[!]"sv;
    case log_level::error:
      return "[-]"sv;
  }
}

} // namespace ai_server::logger

#endif // AI_SERVER_LOGGER_LOG_ITEM_H
