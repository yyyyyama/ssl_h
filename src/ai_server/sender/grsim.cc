#include "grsim.h"

namespace ai_server {
namespace sender {

grsim::grsim(boost::asio::io_service& io_service, const std::string& grsim_addr, short port)
    : udp_sender_(io_service, grsim_addr, port) {}

void grsim::send_command(const model::command& command) {
  ssl_protos::grsim::Packet packet{};

  //パケットに値をセット
  packet.mutable_commands()->set_isteamyellow(true);
  packet.mutable_commands()->set_timestamp(0.0);

  ssl_protos::grsim::Command* grcommand = packet.mutable_commands()->add_robot_commands();

  grcommand->set_id(command.id());

  if (std::get<0>(command.kick_flag()) == model::command::kick_type_t::line)
    grcommand->set_kickspeedx(static_cast<float>(std::get<1>(command.kick_flag())));

  if (std::get<0>(command.kick_flag()) == model::command::kick_type_t::chip ||
      std::get<0>(command.kick_flag()) == model::command::kick_type_t::backspin)
    grcommand->set_kickspeedz(static_cast<float>(std::get<1>(command.kick_flag())));

  grcommand->set_veltangent(
      static_cast<float>(boost::get<model::command::velocity_t>(command.setpoint()).vx));
  grcommand->set_velnormal(
      static_cast<float>(boost::get<model::command::velocity_t>(command.setpoint()).vy));
  grcommand->set_velangular(
      static_cast<float>(boost::get<model::command::velocity_t>(command.setpoint()).omega));

  grcommand->set_spinner(command.dribble() != 0 ? true : false);
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
