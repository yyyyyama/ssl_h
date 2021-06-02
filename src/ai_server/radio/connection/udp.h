#ifndef AI_SERVER_RADIO_CONNECTION_UDP_H
#define AI_SERVER_RADIO_CONNECTION_UDP_H

#include <chrono>
#include <stdexcept>
#include <string_view>
#include <utility>

#define BOOST_COROUTINES_NO_DEPRECATION_WARNING
#include <boost/asio.hpp>
#include <boost/asio/spawn.hpp>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "ai_server/logger/logger.h"
#include "detail/post_and_return_future.h"

namespace ai_server::radio::connection {

/// UDP による送信
class udp_tx {
public:
  /// @param endpoint 送信先のIP・ポート
  /// @param if_addr  通信に使うインターフェイスの IP アドレス
  ///                 (マルチキャストかつ IPv4 のときのみ有効)
  udp_tx(boost::asio::io_context& io_context, const boost::asio::ip::udp::endpoint& endpoint,
         const boost::asio::ip::address_v4& if_addr = boost::asio::ip::address_v4::any())
      : io_context_{io_context},
        work_{boost::asio::make_work_guard(io_context_)},
        endpoint_{endpoint},
        socket_{io_context_, endpoint_.protocol()},
        logger_{"ai_server::radio::connection::udp"},
        timer_{io_context_},
        total_messages_{},
        messages_per_second_{},
        total_errors_{},
        last_sent_{} {
    if (const auto addr = endpoint_.address(); addr.is_v4() && addr.is_multicast()) {
      socket_.set_option(boost::asio::ip::multicast::outbound_interface(if_addr));
    }

    boost::asio::spawn(timer_.get_executor(),
                       [&](auto yield) { count_messages_per_second(yield); });
  }

  /// @param host  送信先のIPアドレス・ホスト名・URL
  /// @param port  送信に使うポート
  udp_tx(boost::asio::io_context& io_context, std::string_view host, unsigned short port)
      : udp_tx(io_context, [&io_context, host, port]() -> boost::asio::ip::udp::endpoint {
          boost::asio::ip::udp::resolver r{io_context};
          auto es = r.resolve(host, ""); // ポートは数値で扱いたいので empty string
          return {es.begin()->endpoint().address(), port};
        }()) {
    logger_.info(fmt::format("{} -> {}", host, endpoint_.address()));
  }

  virtual ~udp_tx() = default;

  template <class Buffer>
  auto send(Buffer buffer) -> decltype(boost::asio::buffer(buffer), void()) {
    boost::asio::spawn(io_context_, [this, buffer = std::move(buffer)](auto yield) {
      boost::system::error_code ec{};
      socket_.async_send_to(boost::asio::buffer(buffer), endpoint_, yield[ec]);
      if (ec) {
        logger_.error(fmt::format("send() failed ({}): {}", endpoint_, ec.message()));
        total_errors_++;
      } else {
        total_messages_++;
        last_sent_ = std::chrono::system_clock::now();
      }
    });
  }

  /// 通信先を取得する
  const boost::asio::ip::udp::endpoint& endpoint() const {
    return endpoint_;
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

protected:
  boost::asio::io_context& io_context_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_;
  boost::asio::ip::udp::endpoint endpoint_;
  boost::asio::ip::udp::socket socket_;

  logger::logger logger_;

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
        logger_.warn(fmt::format("timer is canceled ({})", endpoint_));
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

  boost::asio::steady_timer timer_;

  /// 送信した総メッセージ数
  std::uint64_t total_messages_;
  /// 1秒間に送信したメッセージ数
  std::uint64_t messages_per_second_;
  /// 送信に失敗した数
  std::uint64_t total_errors_;
  /// 最後にメッセージを送信した日時
  std::chrono::system_clock::time_point last_sent_;
};

/// UDP による送信 & 受信
class udp final : public udp_tx {
public:
  /// @param endpoint 送信/受信先のIP・ポート
  /// @param if_addr  通信に使うインターフェイスの IP アドレス
  ///                 (マルチキャストかつ IPv4 のときのみ有効)
  udp(boost::asio::io_context& io_context, const boost::asio::ip::udp::endpoint& endpoint,
      const boost::asio::ip::address_v4& if_addr = boost::asio::ip::address_v4::any())
      : udp_tx(io_context, endpoint, if_addr) {
    const auto addr = endpoint_.address();
    const auto port = endpoint_.port();
    if (addr.is_multicast()) {
      socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
      socket_.bind({endpoint.protocol(), port});
      if (addr.is_v4()) {
        socket_.set_option(boost::asio::ip::multicast::join_group(addr.to_v4(), if_addr));
      } else {
        socket_.set_option(boost::asio::ip::multicast::join_group(addr));
      }
      // 自分で送ったメッセージが受信側に流れないようにする
      socket_.set_option(boost::asio::ip::multicast::enable_loopback(false));
    }
  }

  /// @param host  送信/受信先のIPアドレス・ホスト名・URL
  /// @param port  通信に使うポート
  /// @note        マルチキャストには対応しない
  udp(boost::asio::io_context& io_context, std::string_view host, unsigned short port)
      : udp_tx(io_context, host, port) {}

  template <class Buffer, class Func>
  auto recv(Buffer& buffer, Func func)
      -> decltype(boost::asio::buffer(buffer), func(boost::system::error_code(), std::size_t()),
                  void()) {
    boost::asio::spawn(io_context_, [this, &buffer, func](auto yield) {
      boost::system::error_code ec{};
      boost::asio::ip::udp::endpoint endpoint{};
      const auto size =
          socket_.async_receive_from(boost::asio::buffer(buffer), endpoint, yield[ec]);
      if (ec) {
        logger_.error(fmt::format("recv() failed ({}): {}", endpoint_, ec.message()));
      }
      func(ec, size);
    });
  }
};

} // namespace ai_server::radio::connection

#endif // AI_SERVER_RADIO_CONNECTION_UDP_H
