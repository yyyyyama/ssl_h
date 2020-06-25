#ifndef AI_SERVER_LOGGER_SINK_OSTREAM_H
#define AI_SERVER_LOGGER_SINK_OSTREAM_H

#include <ostream>
#include <string>
#include <string_view>
#include <type_traits>

#include "ai_server/logger/formatter.h"
#include "base.h"

namespace ai_server::logger::sink {

/// ログ発生時に ostream に出力する sink
template <class Stream>
class ostream : public base {
  // Stream は const でない std::basic_ostream<char> が基底クラスであること
  static_assert(
      !std::is_const_v<std::remove_reference_t<Stream>> &&
          std::is_base_of_v<std::basic_ostream<char>, std::remove_reference_t<Stream>>,
      "Stream must be non const, ostream type");

public:
  using ostream_type = Stream;

  /// @param stream   出力先の ostream
  /// @param format   出力フォーマット
  template <class T>
  ostream(T&& stream, std::string_view format)
      : stream_{std::forward<T>(stream)}, format_{format} {}

  /// @param stream   出力先の ostream
  /// @param format   出力フォーマット
  template <class T>
  ostream(T&& stream, std::string_view format, levels_map_type levels)
      : base{std::move(levels)}, stream_{std::forward<T>(stream)}, format_{format} {}

protected:
  void do_log(const log_item& item) override {
    stream_ << format(format_, item) << std::endl;
  }

private:
  /// 出力先の ostream
  ostream_type stream_;
  /// 出力フォーマット
  std::string format_;
};

// コンストラクタの引数から Stream を導出できるようにする
// http://en.cppreference.com/w/cpp/language/class_template_argument_deduction#User-defined_deduction_guides
template <class T>
ostream(T&, ...) -> ostream<T&>;
template <class T>
ostream(T&&, ...) -> ostream<T>;

} // namespace ai_server::logger::sink

#endif // AI_SERVER_LOGGER_SINK_OSTREAM_H
