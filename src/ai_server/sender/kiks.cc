#include "ai_server/sender/kiks.h"

namespace ai_server {
namespace sender {

kiks::kiks(boost::asio::io_service& io_service, const std::string& device)
    : serial_(io_service, device) {
  serial_.set_baud_rate(57600);
}

void kiks::send_command(const model::command& command) {
  data_t data = change_command(command);
  serial_.send(boost::asio::buffer(data));
}

kiks::data_t kiks::change_command(const model::command& command) {
  data_t data{{0, 0, 0, 0, 0, 0, 0, 0, 0, '\r', '\n'}};
  // id
  data[0] = static_cast<uint8_t>(command.id() + 1);
  // kick_flag
  switch (std::get<0>(command.kick_flag())) {
    case model::command::kick_type_t::none:
      data[0] += 0;
      break;
    case model::command::kick_type_t::line:
      data[0] += 48;
      break;
    case model::command::kick_type_t::chip:
    case model::command::kick_type_t::backspin:
      data[0] += 32;
      break;
  }
  // data[1~6]
  const auto& setpoint = command.setpoint();
  if (const auto velocity = boost::get<model::command::velocity_t>(&setpoint)) {
    // rotate
    data[0] += ((velocity->omega >= 0) ? 0 : 128);
    // vec
    auto vec = static_cast<uint16_t>(
        std::sqrt(std::pow(velocity->vx, 2) + std::pow(velocity->vy, 2)) * 1000);
    data[1] = (vec & 0xff00) >> 8;
    data[2] = vec & 0x00ff;
    // dir
    float dir;
    if (velocity->vx < 0) {
      dir = M_PI + std::atan2(velocity->vy * -1, velocity->vx * -1);
    } else {
      dir = std::atan2(velocity->vy, velocity->vx);
      dir += (dir < 0 ? 2 * M_PI : 0);
    }
    data[3] = (static_cast<uint16_t>(dir * 10000) & 0xff00) >> 8;
    data[4] = static_cast<uint16_t>(dir * 10000) & 0x00ff;
    // mrad/s
    auto omega_calc = static_cast<uint16_t>(velocity->omega * 1000);
    data[5]         = (omega_calc & 0xff00) >> 8;
    data[6]         = (omega_calc & 0x00ff);
  }
  // dribble
  data[7] = static_cast<uint8_t>(command.dribble());
  // kick_power
  data[8] = static_cast<uint8_t>(std::get<1>(command.kick_flag()));
  return data;
}
}
}
