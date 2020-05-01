#ifndef AI_SERVER_RADIO_CONNECTION_SERIAL_H
#define AI_SERVER_RADIO_CONNECTION_SERIAL_H

#include <chrono>
#include <string>
#include <utility>

#define BOOST_COROUTINES_NO_DEPRECATION_WARNING
#include <boost/asio.hpp>
#include <boost/asio/spawn.hpp>

#include "ai_server/logger/logger.h"
#include "detail/post_and_return_future.h"

namespace ai_server::radio::connection {

class serial {
public:
  using baud_rate      = boost::asio::serial_port::baud_rate;
  using flow_control   = boost::asio::serial_port::flow_control;
  using parity         = boost::asio::serial_port::parity;
  using stop_bits      = boost::asio::serial_port::stop_bits;
  using character_size = boost::asio::serial_port::character_size;

  template <class... Options>
  serial(boost::asio::io_context& io_context, const std::string& device, const Options&... opts)
      : device_{device},
        total_messages_{},
        messages_per_second_{},
        total_errors_{},
        last_sent_{},
        io_context_{io_context},
        work_{boost::asio::make_work_guard(io_context_)},
        serial_{io_context_, device_},
        timer_{io_context_} {
    (serial_.set_option(opts), ...);
    boost::asio::spawn(timer_.get_executor(),
                       [&](auto yield) { count_messages_per_second(yield); });
  }

  template <class Buffer>
  auto send(Buffer buffer) -> decltype(boost::asio::buffer(buffer), void()) {
    boost::asio::spawn(io_context_, [this, buffer = std::move(buffer)](auto yield) {
      boost::system::error_code ec{};
      serial_.async_write_some(boost::asio::buffer(buffer), yield[ec]);
      if (ec) {
        logger_.error("send() failed (" + device_ + "): " + ec.message());
        total_errors_++;
      } else {
        total_messages_++;
        last_sent_ = std::chrono::system_clock::now();
      }
    });
  }

  /// @brief 送信した総メッセージ数を取得する
  std::uint64_t total_messages() const {
    return detail::post_and_return_future(io_context_, [this] { return total_messages_; })
        .get();
  }

  /// @brief 1秒間に送信したメッセージ数を取得する
  std::uint64_t messages_per_second() const {
    return detail::post_and_return_future(io_context_, [this] { return messages_per_second_; })
        .get();
  }

  /// @brief 送信に失敗した数を取得する
  std::uint64_t total_errors() const {
    return detail::post_and_return_future(io_context_, [this] { return total_errors_; }).get();
  }

  /// @brief 最後にメッセージを送信した日時を取得する
  std::chrono::system_clock::time_point last_sent() const {
    return detail::post_and_return_future(io_context_, [this] { return last_sent_; }).get();
  }

private:
  void count_messages_per_second(boost::asio::yield_context yield) {
    using namespace std::chrono_literals;

    boost::system::error_code ec;

    auto prev_total_messages = total_messages_;

    timer_.expires_after(1s);

    for (;;) {
      // 設定された時刻まで待つ
      timer_.async_wait(yield[ec]);
      if (ec) {
        logger_.warn("timer is canceled (" + device_ + ")");
        break;
      }

      // 1秒間で受信したメッセージ数を求めて更新する
      const auto tm        = total_messages_;
      messages_per_second_ = tm - prev_total_messages;
      prev_total_messages  = tm;

      // 次に更新を行う時刻 (1秒後) を設定
      timer_.expires_after(1s);
    }
  }

  std::string device_;

  /// 送信した総メッセージ数
  std::uint64_t total_messages_;
  /// 1秒間に送信したメッセージ数
  std::uint64_t messages_per_second_;
  /// 送信に失敗した数
  std::uint64_t total_errors_;
  /// 最後にメッセージを送信した日時
  std::chrono::system_clock::time_point last_sent_;

  boost::asio::io_context& io_context_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_;
  boost::asio::serial_port serial_;
  boost::asio::steady_timer timer_;

  logger::logger_for<serial> logger_;
};

} // namespace ai_server::radio::connection

#endif // AI_SERVER_RADIO_CONNECTION_SERIAL_H
