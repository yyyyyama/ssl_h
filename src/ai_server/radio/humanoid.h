#ifndef AI_SERVER_RADIO_HUMANOID_H
#define AI_SERVER_RADIO_HUMANOID_H

#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include <boost/math/constants/constants.hpp>

#include "ai_server/util/math/angle.h"
#include "base/base.h"

namespace ai_server::radio {

template <class Connection>
class humanoid : public base::command {
public:
  humanoid(std::unique_ptr<Connection> connection) : connection_{std::move(connection)} {}

  const Connection& connection() const {
    return *connection_;
  }

  void send([[maybe_unused]] model::team_color color, unsigned int id,
            const model::command::kick_flag_t& kick_flag, int dribble, double vx, double vy,
            double omega) override {
    std::vector<std::uint8_t> data(11);

    data[0] = (id + 1) & 0b1111;

    switch (std::get<0>(kick_flag)) {
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

    data[0] |= omega >= 0 ? 0b00000000 : 0b10000000;

    auto vel = static_cast<std::uint16_t>(std::hypot(vx, vy));
    data[1]  = (vel & 0xff00) >> 8;
    data[2]  = (vel & 0x00ff);

    namespace bmc = boost::math::double_constants;

    auto dir = ai_server::util::math::wrap_to_2pi(std::atan2(vy, vx) + bmc::half_pi);
    data[3]  = (static_cast<std::uint16_t>((dir / bmc::two_pi) * 0xffff) & 0xff00) >> 8;
    data[4]  = (static_cast<std::uint16_t>((dir / bmc::two_pi) * 0xffff) & 0x00ff);

    auto o  = static_cast<std::uint16_t>(std::abs(omega) * 1000);
    data[5] = (o & 0xff00) >> 8;
    data[6] = (o & 0x00ff);

    data[7] = static_cast<std::uint8_t>(std::abs(dribble + 3));
    data[8] = static_cast<std::uint8_t>(std::get<1>(kick_flag));

    data[9]  = '\r';
    data[10] = '\n';

    connection_->send(std::move(data));
  }

  void send([[maybe_unused]] model::team_color color, unsigned int id,
            std::shared_ptr<model::motion::base> motion) override {
    if (motion) {
      std::cout << id << ": " << static_cast<int>(motion->motion_id()) << std::endl;
      std::vector<std::uint8_t> data(2);
      data[0] = id;
      data[1] = motion->motion_id();
      connection_->send(std::move(data));
    }
  }

protected:
  std::unique_ptr<Connection> connection_;
};

} // namespace ai_server::radio

#endif // AI_SERVER_RADIO_HUMANOID_H
