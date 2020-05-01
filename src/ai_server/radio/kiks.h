#ifndef AI_SERVER_RADIO_KIKS_H
#define AI_SERVER_RADIO_KIKS_H

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <boost/math/constants/constants.hpp>

#include "ai_server/util/math/angle.h"
#include "base/base.h"

namespace ai_server::radio {

template <class Connection>
class kiks : public base::command {
public:
  kiks(std::unique_ptr<Connection> connection) : connection_{std::move(connection)} {}

  const Connection& connection() const {
    return *connection_;
  }

  void send([[maybe_unused]] model::team_color color, const model::command& command) override {
    std::vector<std::uint8_t> data(11);

    data[0] = (command.id() + 1) & 0b1111;

    switch (std::get<0>(command.kick_flag())) {
      case model::command::kick_type_t::line:
        data[0] |= 0b00110000;
        break;
      case model::command::kick_type_t::chip:
      case model::command::kick_type_t::backspin:
        data[0] |= 0b00100000;
        break;
      default:
        data[0] |= 0b00000000;
    }

    if (const auto& setpoint = command.setpoint();
        const auto velocity  = std::get_if<model::command::velocity_t>(&setpoint)) {
      data[0] |= (velocity->omega >= 0) ? 0b00000000 : 0b10000000;

      auto vel = static_cast<std::uint16_t>(std::hypot(velocity->vx, velocity->vy));
      data[1]  = (vel & 0xff00) >> 8;
      data[2]  = (vel & 0x00ff);

      namespace bmc = boost::math::double_constants;

      auto dir = ai_server::util::math::wrap_to_2pi(std::atan2(velocity->vy, velocity->vx) +
                                                    bmc::half_pi);
      data[3]  = (static_cast<std::uint16_t>((dir / bmc::two_pi) * 0xffff) & 0xff00) >> 8;
      data[4]  = (static_cast<std::uint16_t>((dir / bmc::two_pi) * 0xffff) & 0x00ff);

      auto omega = static_cast<std::uint16_t>(std::abs(velocity->omega) * 1000);
      data[5]    = (omega & 0xff00) >> 8;
      data[6]    = (omega & 0x00ff);
    }

    data[7] = static_cast<std::uint8_t>(std::abs(command.dribble() + 3));
    data[8] = static_cast<std::uint8_t>(std::get<1>(command.kick_flag()));

    data[9]  = '\r';
    data[10] = '\n';

    connection_->send(std::move(data));
  }

protected:
  std::unique_ptr<Connection> connection_;
};

} // namespace ai_server::radio

#endif // AI_SERVER_RADIO_KIKS_H
