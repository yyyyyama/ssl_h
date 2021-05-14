#ifndef AI_SERVER_RECEIVER_REFBOX_H
#define AI_SERVER_RECEIVER_REFBOX_H

#include <chrono>
#include <cstdint>
#include <string>
#include <shared_mutex>

#include <boost/asio.hpp>
#include <boost/signals2.hpp>

#include "ai_server/logger/logger.h"
#include "ai_server/util/net/multicast/receiver.h"

// 前方宣言
namespace ssl_protos {
namespace gc {
class Referee;
}
} // namespace ssl_protos

namespace ai_server {
namespace receiver {

class refbox {
  mutable std::shared_mutex mutex_;

public:
  /// データ受信時に発火する signalの型
  using receive_signal_type = boost::signals2::signal<void(const ssl_protos::gc::Referee&)>;
  /// receive_signal_type に登録する slot の型
  using receive_slot_type          = typename receive_signal_type::slot_type;
  using receive_extended_slot_type = typename receive_signal_type::extended_slot_type;

  /// エラー時に発火する signal の型
  using error_signal_type = boost::signals2::signal<void(void)>;
  /// error_signal_type に登録する slot の型
  using error_slot_type          = typename error_signal_type::slot_type;
  using error_extedned_slot_type = typename error_signal_type::extended_slot_type;

  refbox(boost::asio::io_context& io_context, const std::string& listen_addr,
         const std::string& multicast_addr, unsigned short port);

  /// @brief                  データ受信時に slot が呼ばれるようにする
  /// @param slot             データ受信時に呼びたい関数オブジェクト
  boost::signals2::connection on_receive(const receive_slot_type& slot);
  /// @brief                  データ受信時に slot が呼ばれるようにする
  /// @param slot             データ受信時に呼びたい関数オブジェクト
  boost::signals2::connection on_receive_extended(const receive_extended_slot_type& slot);

  /// @brief                  エラー時に slot が呼ばれるようにする
  /// @param slot             エラー時に呼びたい関数オブジェクト
  boost::signals2::connection on_error(const error_slot_type& slot);
  /// @brief                  エラー時に slot が呼ばれるようにする
  /// @param slot             エラー時に呼びたい関数オブジェクト
  boost::signals2::connection on_error_extended(const error_extedned_slot_type& slot);

  /// @brief 受信した総メッセージ数を取得する
  std::uint64_t total_messages() const;

  /// @brief 1秒間に受信したメッセージ数を取得する
  std::uint64_t messages_per_second() const;

  /// @brief 受信したメッセージのパースに失敗した数を取得する
  std::uint64_t parse_error() const;

  /// @brief 最後にメッセージを受信した日時を取得する
  std::chrono::system_clock::time_point last_updated() const;

private:
  /// @brief receiver_ が新しいメッセージを受信したときに呼ばれる関数
  void handle_receive(const util::net::multicast::receiver::buffer_t& buffer,
                      std::size_t length, std::uint64_t total_messages,
                      std::chrono::system_clock::time_point time);

  /// @brief receiver_ で受信状況が更新されたときに呼ばれる関数
  void handle_status_updated(std::uint64_t messages_per_second);

  /// @brief receiver_ でエラーが発生したときに呼ばれる関数
  void handle_error(const boost::system::error_code& ec);

  /// 受信した総メッセージ数
  std::uint64_t total_messages_;
  /// 1秒間に受信したメッセージ数
  std::uint64_t messages_per_second_;
  /// 受信したメッセージのパースに失敗した数
  std::uint64_t parse_error_;
  /// 最後にメッセージを受信した日時
  std::chrono::system_clock::time_point last_updated_;

  util::net::multicast::receiver receiver_;

  receive_signal_type receive_signal_;
  error_signal_type error_signal_;

  logger::logger_for<refbox> logger_;
};
} // namespace receiver
} // namespace ai_server

#endif
