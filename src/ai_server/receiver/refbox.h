#ifndef AI_SERVER_RECEIVER_REFBOX_H
#define AI_SERVER_RECEIVER_REFBOX_H

#include <string>
#include <boost/asio.hpp>
#include <boost/signals2.hpp>

#include "ai_server/model/refbox.h"
#include "ai_server/util/net/multicast/receiver.h"

#include "ssl-protos/refbox/referee.pb.h"

namespace ai_server {
namespace receiver {

class refbox {
  // データ受信時に呼ぶシグナルの型
  using receive_signal_t = boost::signals2::signal<void(const ssl_protos::refbox::Referee&)>;
  // エラー時に呼ぶシグナルの型
  using error_signal_t = boost::signals2::signal<void(void)>;

public:
  refbox(boost::asio::io_service& io_service, const std::string& listen_addr,
         const std::string& multicast_addr, short port);
  boost::signals2::connection on_receive(const receive_signal_t::slot_type& slot);
  boost::signals2::connection on_error(const error_signal_t::slot_type& slot);
  const model::refbox& refbox_data() const;

private:
  void parse_packet(const util::net::multicast::receiver::buffer_t& buffer, std::size_t size);
  util::net::multicast::receiver receiver_;
  model::refbox refbox_data_;

  receive_signal_t received_;
  error_signal_t errored_;
};
}
}

#endif