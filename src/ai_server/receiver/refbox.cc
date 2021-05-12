#include "ssl-protos/gc_referee_message.pb.h"

#include "refbox.h"

namespace ai_server {
namespace receiver {

refbox::refbox(boost::asio::io_context& io_context, const std::string& listen_addr,
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

boost::signals2::connection refbox::on_receive(const receive_slot_type& slot) {
  return receive_signal_.connect(slot);
}

boost::signals2::connection refbox::on_receive_extended(
    const receive_extended_slot_type& slot) {
  return receive_signal_.connect_extended(slot);
}

boost::signals2::connection refbox::on_error(const error_slot_type& slot) {
  return error_signal_.connect(slot);
}

boost::signals2::connection refbox::on_error_extended(const error_extedned_slot_type& slot) {
  return error_signal_.connect_extended(slot);
}

std::uint64_t refbox::total_messages() const {
  std::shared_lock lock{mutex_};
  return total_messages_;
}

std::uint64_t refbox::messages_per_second() const {
  std::shared_lock lock{mutex_};
  return messages_per_second_;
}

std::uint64_t refbox::parse_error() const {
  std::shared_lock lock{mutex_};
  return parse_error_;
}

std::chrono::system_clock::time_point refbox::last_updated() const {
  std::shared_lock lock{mutex_};
  return last_updated_;
}

void refbox::handle_receive(const util::net::multicast::receiver::buffer_t& buffer,
                            std::size_t size, std::uint64_t total_messages,
                            std::chrono::system_clock::time_point time) {
  ssl_protos::gc::Referee packet;

  if (packet.ParseFromArray(buffer.data(), size)) {
    {
      std::unique_lock lock{mutex_};
      total_messages_ = total_messages;
      last_updated_   = time;
    }

    receive_signal_(packet);
  } else {
    {
      std::unique_lock lock{mutex_};
      total_messages_ = total_messages;
      last_updated_   = time;
      parse_error_ += 1;

      logger_.warn("failed to parse message " + std::to_string(total_messages_));
    }

    error_signal_();
  }
}

void refbox::handle_status_updated(std::uint64_t messages_per_second) {
  std::unique_lock lock{mutex_};
  messages_per_second_ = messages_per_second;
}

void refbox::handle_error(const boost::system::error_code& ec) {
  logger_.error(ec.message());
  error_signal_();
}

} // namespace receiver
} // namespace ai_server
