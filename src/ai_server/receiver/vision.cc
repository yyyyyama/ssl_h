#include "ssl-protos/vision_wrapper.pb.h"

#include "vision.h"

namespace ai_server {
namespace receiver {

vision::vision(boost::asio::io_context& io_context, const std::string& listen_addr,
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

boost::signals2::connection vision::on_receive(const receive_slot_type& slot) {
  return receive_signal_.connect(slot);
}

boost::signals2::connection vision::on_receive_extended(
    const receive_extended_slot_type& slot) {
  return receive_signal_.connect_extended(slot);
}

boost::signals2::connection vision::on_error(const error_slot_type& slot) {
  return error_signal_.connect(slot);
}

boost::signals2::connection vision::on_error_extended(const error_extedned_slot_type& slot) {
  return error_signal_.connect_extended(slot);
}

std::uint64_t vision::total_messages() const {
  std::shared_lock lock{mutex_};
  return total_messages_;
}

std::uint64_t vision::messages_per_second() const {
  std::shared_lock lock{mutex_};
  return messages_per_second_;
}

std::uint64_t vision::parse_error() const {
  std::shared_lock lock{mutex_};
  return parse_error_;
}

std::chrono::system_clock::time_point vision::last_updated() const {
  std::shared_lock lock{mutex_};
  return last_updated_;
}

void vision::handle_receive(const util::net::multicast::receiver::buffer_t& buffer,
                            std::size_t size, std::uint64_t total_messages,
                            std::chrono::system_clock::time_point time) {
  ssl_protos::vision::Packet packet;

  // パケットをパース
  if (packet.ParseFromArray(buffer.data(), size)) {
    {
      std::unique_lock lock{mutex_};
      total_messages_ = total_messages;
      last_updated_   = time;
    }

    if (packet.has_detection()) {
      auto detection = packet.mutable_detection();
      adjust_detection_timestamps(*detection, time);
    }

    // 成功したら登録された関数を呼び出す
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

void vision::handle_status_updated(std::uint64_t messages_per_second) {
  std::unique_lock lock{mutex_};
  messages_per_second_ = messages_per_second;
}

void vision::handle_error(const boost::system::error_code& ec) {
  logger_.error(ec.message());
  error_signal_();
}

void vision::adjust_detection_timestamps(ssl_protos::vision::Frame& detection,
                                         std::chrono::system_clock::time_point time) {
  constexpr auto den = std::chrono::system_clock::duration::period::den;
  constexpr auto num = std::chrono::system_clock::duration::period::num;

  // time を Vision で使われる形式 (double で単位が秒) に変換
  const auto te = time.time_since_epoch();
  const auto tt = static_cast<double>(te.count() * num) / den;

  // t_sent と ai-server 側の時刻の差を求める
  const double d = tt - detection.t_sent();

  // カメラID毎の平均時差を更新する
  auto& [n, m] = time_diff_map_[detection.camera_id()];
  m += (d - m) / ++n;

  detection.set_t_capture(detection.t_capture() + m);
  detection.set_t_sent(detection.t_sent() + m);
}

} // namespace receiver
} // namespace ai_server
