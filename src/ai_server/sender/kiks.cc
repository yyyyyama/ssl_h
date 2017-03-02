#include <boost/math/constants/constants.hpp>
#include <cmath>

#include "ai_server/sender/kiks.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace sender {

kiks::kiks(boost::asio::io_service& io_service, const std::string& device)
    : serial_(io_service, device) {
  serial_.set_baud_rate(57600);
}

void kiks::send_command(const model::command& command) {
  auto data = to_data_t(command);
  serial_.send(data);
}

kiks::data_t kiks::to_data_t(const model::command& command) {
  using boost::math::constants::two_pi;
  using boost::math::constants::half_pi;

  data_t data{{0, 0, 0, 0, 0, 0, 0, 0, 0, '\r', '\n'}};
  // id
  data[0] = (0b00001111 & (command.id() + 1));
  // kick_flag
  switch (std::get<0>(command.kick_flag())) {
    case model::command::kick_type_t::none:
      data[0] |= 0b00000000;
      break;
    case model::command::kick_type_t::line:
      data[0] |= 0b00110000;
      break;
    case model::command::kick_type_t::chip:
    case model::command::kick_type_t::backspin:
      data[0] |= 0b00100000;
      break;
  }
  // data[1~6]
  const auto& setpoint = command.setpoint();
  if (const auto velocity = boost::get<model::command::velocity_t>(&setpoint)) {
    // rotate
    data[0] |= ((velocity->omega >= 0) ? 0b00000000 : 0b10000000);
    // vec
    auto vec = static_cast<uint16_t>(std::hypot(velocity->vx, velocity->vy) * 1000);
    data[1]  = (vec & 0xff00) >> 8;
    data[2]  = (vec & 0x00ff);
    // dir
    auto dir = ai_server::util::wrap_to_2pi(std::atan2(velocity->vy, velocity->vx) +
                                            half_pi<double>());
    data[3] = (static_cast<uint16_t>((dir / two_pi<double>()) * 0xffff) & 0xff00) >> 8;
    data[4] = (static_cast<uint16_t>((dir / two_pi<double>()) * 0xffff) & 0x00ff);
    // mrad/s
    auto omega_calc = static_cast<uint16_t>(std::abs(velocity->omega) * 1000);
    data[5]         = (omega_calc & 0xff00) >> 8;
    data[6]         = (omega_calc & 0x00ff);
  }
  // dribble
  data[7] = static_cast<uint8_t>(std::abs(command.dribble()));
  // kick_power
  data[8] = static_cast<uint8_t>(std::get<1>(command.kick_flag()));
  return data;
}
}
}
