#include <functional>
#include "refbox.h"

namespace ai_server {
namespace receiver {

refbox::refbox(boost::asio::io_service& io_service, const std::string& listen_addr,
               const std::string& multicast_addr, short port)
    : receiver_(io_service, listen_addr, multicast_addr, port), refbox_data_() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  receiver_.on_receive(
      std::bind(&refbox::parse_packet, this, std::placeholders::_1, std::placeholders::_2));
}

boost::signals2::connection refbox::on_receive(const receive_signal_t::slot_type& slot) {
  return received_.connect(slot);
}

boost::signals2::connection refbox::on_error(const error_signal_t::slot_type& slot) {
  return errored_.connect(slot);
}

const model::refbox& refbox::refbox_data() const {
  return refbox_data_;
}

void refbox::parse_packet(const util::net::multicast::receiver::buffer_t& buffer,
                          std::size_t size) {
  ssl_protos::refbox::Referee packet;

  if (packet.ParseFromArray(buffer.data(), size)) {
    received_(packet);

    model::refbox::team_info yellow{packet.yellow().name()};
    model::refbox::team_info blue{packet.blue().name()};

    refbox_data_.set_packet_timestamp(packet.packet_timestamp());
    refbox_data_.set_stage_time_left(packet.stage_time_left());
    yellow.set_score(packet.yellow().score());
    yellow.set_goalie(packet.yellow().goalie());
    yellow.set_red_cards(packet.yellow().red_cards());
    yellow.set_yellow_cards(packet.yellow().yellow_cards());
    yellow.set_yellow_card_times(packet.yellow().yellow_card_times(0));
    yellow.set_timeouts(packet.yellow().timeouts());
    yellow.set_timeout_time(packet.yellow().timeout_time());
    refbox_data_.set_team_yellow(yellow);
    blue.set_score(packet.blue().score());
    blue.set_goalie(packet.blue().goalie());
    blue.set_red_cards(packet.blue().red_cards());
    blue.set_yellow_cards(packet.blue().yellow_cards());
    blue.set_yellow_card_times(packet.blue().yellow_card_times(0));
    blue.set_timeouts(packet.blue().timeouts());
    blue.set_timeout_time(packet.blue().timeout_time());
    refbox_data_.set_team_blue(blue);
    refbox_data_.set_stage((model::refbox::stage_name)packet.stage());
    refbox_data_.set_command((model::refbox::game_command)packet.command());
  } else {
    errored_();
  }
}
}
}