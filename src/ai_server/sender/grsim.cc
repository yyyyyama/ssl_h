#include <boost/variant.hpp>
#include <ostream>
#include <tuple>

#include "grsim.h"

namespace ai_server {
namespace sender {

grsim::grsim(boost::asio::io_service& io_service, const std::string& grsim_addr, short port)
    : udp_sender_(io_service, grsim_addr, port) {
  //プロトコルバッファのバージョン確認
  GOOGLE_PROTOBUF_VERIFY_VERSION;
}

void grsim::send_command(const model::command& command) {
  ssl_protos::grsim::Packet packet{};

  //パケットに値をセット
  auto commands = packet.mutable_commands();
  commands->set_isteamyellow(true);
  commands->set_timestamp(0.0);

  auto grcommand = commands->add_robot_commands();

  grcommand->set_id(command.id());

  const auto& kick_flag  = std::get<0>(command.kick_flag());
  const auto& kick_power = std::get<1>(command.kick_flag());

  using kick_type_t = model::command::kick_type_t;

  if (kick_flag == kick_type_t::line) {
    grcommand->set_kickspeedx(kick_power);
    grcommand->set_kickspeedz(0);
  } else if (kick_flag == kick_type_t::chip || kick_flag == kick_type_t::backspin) {
    grcommand->set_kickspeedx(0);
    grcommand->set_kickspeedz(kick_power);
  } else {
    grcommand->set_kickspeedx(0);
    grcommand->set_kickspeedz(0);
  }

  const auto& setpoint = command.setpoint();
  if (const auto velocity = boost::get<model::command::velocity_t>(&setpoint)) {
    // velocity_tへのキャストが成功した時
    grcommand->set_veltangent(velocity->vx);
    grcommand->set_velnormal(velocity->vy);
    grcommand->set_velangular(velocity->omega);
  } else {
    // velocity_tへのキャストが失敗した時
    grcommand->set_veltangent(0);
    grcommand->set_velnormal(0);
    grcommand->set_velangular(0);
  }

  grcommand->set_spinner(command.dribble() != 0);
  grcommand->set_wheelsspeed(false);

  //送信バッファと、それに書き込むstreamクラスのオブジェクトを作成
  boost::asio::streambuf buf;
  std::ostream os(&buf);

  // packetをシリアライズし、結果をos経由でbufに書き込む
  packet.SerializeToOstream(&os);

  // bufを送信
  udp_sender_.send(buf.data());
}
} // namespace sender
} // namespace ai_server
