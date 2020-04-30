#ifndef AI_SERVER_LOGGER_DETAIL_FORMATTER_H
#define AI_SERVER_LOGGER_DETAIL_FORMATTER_H

#include <chrono>
#include <ctime>
#include <string_view>
#include <utility>

#include <fmt/chrono.h>
#include <fmt/format.h>

#include "ai_server/logger/log_item.h"
#include "formatter.h"

namespace ai_server::logger::detail {

struct base {
  const log_item& item;
};
struct elapsed : base {};
struct level : base {};
struct level_simple : base {};
struct time : base {};
struct thread_id : base {};

} // namespace ai_server::logger::detail

namespace fmt {

using namespace std::chrono_literals;

namespace al = ::ai_server::logger;

template <>
struct formatter<al::detail::elapsed> {
  constexpr auto parse(format_parse_context& ctx) {
    return ctx.begin();
  }

  template <class FormatContext>
  auto format(const al::detail::elapsed& s, FormatContext& ctx) {
    const auto n = (s.item.time_stamp - elapsed_time_offset) / 1us;
    return format_to(ctx.out(), "{:6d}.{:06d}", n / 1000000, n % 1000000);
  }
  static const std::chrono::steady_clock::time_point elapsed_time_offset;
};

template <>
struct formatter<al::detail::level> : formatter<std::string_view> {
  template <class FormatContext>
  auto format(const al::detail::level& s, FormatContext& ctx) {
    return formatter<std::string_view>::format(al::log_level_to_string(s.item.level), ctx);
  }
};

template <>
struct formatter<al::detail::level_simple> : formatter<std::string_view> {
  template <class FormatContext>
  auto format(const al::detail::level_simple& s, FormatContext& ctx) {
    return formatter<std::string_view>::format(al::log_level_to_symbol(s.item.level), ctx);
  }
};

template <>
struct formatter<al::detail::time> : formatter<std::tm> {
  template <class FormatContext>
  auto format(const al::detail::time&, FormatContext& ctx) {
    const auto t = std::time(nullptr);
    return formatter<std::tm>::format(*std::localtime(&t), ctx);
  }
};

using thread_id_type = decltype(std::declval<al::log_item>().thread_id);

template <>
struct formatter<al::detail::thread_id> : formatter<thread_id_type> {
  template <class FormatContext>
  auto format(const al::detail::thread_id& s, FormatContext& ctx) {
    return formatter<thread_id_type>::format(s.item.thread_id, ctx);
  }
};

} // namespace fmt

#endif // AI_SERVER_LOGGER_DETAIL_FORMATTER_H
