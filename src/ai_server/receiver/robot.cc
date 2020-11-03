#include "robot.h"

namespace ai_server {
namespace receiver {

robot::robot(boost::asio::io_context& io_context, const std::string& listen_addr,
             const std::string& multicast_addr, unsigned short port)
    : total_messages_{},
      messages_per_second_{},
      parse_error_{},
      last_updated_{},
      receiver_{io_context, listen_addr, multicast_addr, port} {
  // multicast receiver のコールバック関数を登録する
  receiver_.on_receive(
      [&](auto&&... args) { handle_receive(std::forward<decltype(args)>(args)...); });
  receiver_.on_status_updated([&](auto mps) { handle_status_updated(mps); });
  receiver_.on_error([&](const auto& ec) { handle_error(ec); });
}

boost::signals2::connection robot::on_receive(const receive_slot_type& slot) {
  return receive_signal_.connect(slot);
}

boost::signals2::connection robot::on_error(const error_slot_type& slot) {
  return error_signal_.connect(slot);
}

std::uint64_t robot::total_messages() const {
  std::shared_lock lock{mutex_};
  return total_messages_;
}

std::uint64_t robot::messages_per_second() const {
  std::shared_lock lock{mutex_};
  return messages_per_second_;
}

std::uint64_t robot::parse_error() const {
  std::shared_lock lock{mutex_};
  return parse_error_;
}

std::chrono::system_clock::time_point robot::last_updated() const {
  std::shared_lock lock{mutex_};
  return last_updated_;
}

void robot::handle_receive(const util::net::multicast::receiver::buffer_t& buffer,
                           std::size_t size, std::uint64_t total_messages,
                           std::chrono::system_clock::time_point time) {
  std::vector<std::uint8_t> packet;
  for (std::size_t i = 0; i < size; ++i) packet.push_back(buffer[i]);
  receive_signal_(packet);
}

void robot::handle_status_updated(std::uint64_t messages_per_second) {
  std::unique_lock lock{mutex_};
  messages_per_second_ = messages_per_second;
}

void robot::handle_error(const boost::system::error_code& ec) {
  logger_.error(ec.message());
  error_signal_();
}

} // namespace receiver
} // namespace ai_server
