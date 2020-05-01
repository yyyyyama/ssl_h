#ifndef AI_SERVER_RADIO_GRSIM_H
#define AI_SERVER_RADIO_GRSIM_H

#include <memory>

#include <boost/math/constants/constants.hpp>

#include "base/base.h"

#include "ssl-protos/grsim/packet.pb.h"

namespace ai_server::radio {

template <class Connection>
class grsim : public base::command, public base::simulator {
public:
  grsim(std::unique_ptr<Connection> connection) : connection_{std::move(connection)} {}

  const Connection& connection() const {
    return *connection_;
  }

  void send(model::team_color color, const model::command& command) override {
    ssl_protos::grsim::Packet packet{};

    auto commands = packet.mutable_commands();
    commands->set_isteamyellow(color == model::team_color::yellow);
    commands->set_timestamp(0.0);

    auto gr_cmd = commands->add_robot_commands();
    convert(command, *gr_cmd);

    connection_->send(packet.SerializeAsString());
  }

  void set_ball_position(double x, double y) override {
    ssl_protos::grsim::Packet packet{};

    auto b = packet.mutable_replacement()->mutable_ball();
    b->set_x(x / 1000);
    b->set_y(y / 1000);
    b->set_vx(0);
    b->set_vy(0);

    connection_->send(packet.SerializeAsString());
  }

  void set_robot_position(model::team_color color, unsigned int id, double x, double y,
                          double theta) override {
    ssl_protos::grsim::Packet packet{};

    auto r = packet.mutable_replacement()->add_robots();
    r->set_id(id);
    r->set_x(x / 1000);
    r->set_y(y / 1000);
    r->set_dir(theta * 180.0 / boost::math::double_constants::pi);
    r->set_yellowteam(color == model::team_color::yellow);

    connection_->send(packet.SerializeAsString());
  }

protected:
  static void convert(const model::command& command, ssl_protos::grsim::Command& gr_cmd) {
    gr_cmd.set_id(command.id());

    switch (const auto& [type, power] = command.kick_flag(); type) {
      case model::command::kick_type_t::line:
        gr_cmd.set_kickspeedx(power);
        gr_cmd.set_kickspeedz(0);
        break;

      case model::command::kick_type_t::chip:
      case model::command::kick_type_t::backspin:
        gr_cmd.set_kickspeedx(0);
        gr_cmd.set_kickspeedz(power);
        break;

      default:
        gr_cmd.set_kickspeedx(0);
        gr_cmd.set_kickspeedz(0);
    }

    if (const auto& setpoint = command.setpoint();
        const auto velocity  = std::get_if<model::command::velocity_t>(&setpoint)) {
      // velocity_tへのキャストが成功した時
      gr_cmd.set_veltangent(velocity->vx / 1000);
      gr_cmd.set_velnormal(velocity->vy / 1000);
      gr_cmd.set_velangular(velocity->omega);
    } else {
      // velocity_tへのキャストが失敗した時
      gr_cmd.set_veltangent(0);
      gr_cmd.set_velnormal(0);
      gr_cmd.set_velangular(0);
    }

    gr_cmd.set_spinner(command.dribble() != 0);
    gr_cmd.set_wheelsspeed(false);
  }

  std::unique_ptr<Connection> connection_;
};

} // namespace ai_server::radio

#endif // AI_SERVER_RADIO_GRSIM_H
