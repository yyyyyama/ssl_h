#ifndef AI_SERVER_RADIO_SSL_SIMPROTO_H
#define AI_SERVER_RADIO_SSL_SIMPROTO_H

#include <array>
#include <cmath>
#include <memory>
#include <string_view>

#include <boost/math/constants/constants.hpp>
#include <boost/system/error_code.hpp>

#include <fmt/format.h>

#include "ssl-protos/simulation_control.pb.h"
#include "ssl-protos/simulation_robot_control.pb.h"
#include "ssl-protos/simulation_robot_feedback.pb.h"

#include "base/base.h"

#include "ai_server/logger/logger.h"
#include "ai_server/model/team_color.h"
#include "ai_server/util/kick/convert.h"

// SSL Simulation Protocol (https://github.com/RoboCup-SSL/ssl-simulation-protocol) の radio
// Simulation Control, Robot Control Blue/Yellow それぞれが独立したポートで通信するのを想定した
// プロトコルであるため、コンストラクタで connection を複数取る実装となっている。
// また SSL Simulation Protocol では双方向で通信するため、通信には recv() が実装された
// connection が必要である。

namespace ai_server::radio::ssl_simproto {

/// 各チームのロボットへ命令を送る (Robot Control Blue/Yellow)
template <class Connection>
class robot : public base::command {
public:
  /// @param connection_blue    ロボットのコマンド送信・ステータス受信に使う connection (Blue)
  /// @param connection_yellow  ロボットのコマンド送信・ステータス受信に使う connection (Yellow)
  robot(std::unique_ptr<Connection> connection_blue,
        std::unique_ptr<Connection> connection_yellow)
      : connection_blue_{std::move(connection_blue)},
        connection_yellow_{std::move(connection_yellow)} {
    recv<model::team_color::blue>();
    recv<model::team_color::yellow>();
  }

  void send(model::team_color color, unsigned int id,
            const model::command::kick_flag_t& kick_flag, int dribble, double vx, double vy,
            double omega) override {
    ssl_protos::simulation::RobotControl packet{};

    auto sim_cmd = packet.add_robot_commands();
    sim_cmd->set_id(id);

    auto mc = sim_cmd->mutable_move_command();
    {
      auto lv = mc->mutable_local_velocity();
      lv->set_left(vy / 1000);
      lv->set_forward(vx / 1000);
      lv->set_angular(omega);
    }

    using kick_types = model::command::kick_type_t;
    if (const auto& [type, power] = kick_flag; type == kick_types::line) {
      sim_cmd->set_kick_speed(util::kick::power_to_speed(power) / 1000);
      sim_cmd->set_kick_angle(0);
    } else if (type == kick_types::chip || type == kick_types::backspin) {
      sim_cmd->set_kick_speed(util::kick::power_to_speed(power) / 1000);
      sim_cmd->set_kick_angle(60); // deg
    }

    if (dribble != 0) {
      // KIKS の機体は 200.0 * dribble [rpm] くらいらしい
      // https://github.com/kiksworks/ai-server/pull/744#discussion_r648836807
      // TODO: シミュレータの挙動も見ながら必要に応じて調整
      sim_cmd->set_dribbler_speed(200.0 * std::abs(dribble)); // rpm
    }

    connection(color).send(packet.SerializeAsString());
  }

private:
  template <model::team_color Color>
  void recv() {
    connection(Color).recv(buf(Color), [this](auto&&... args) {
      this->handle_recv<Color>(std::forward<decltype(args)>(args)...);
    });
  }

  template <model::team_color Color>
  void handle_recv(const boost::system::error_code& ec, std::size_t length) {
    if (ec) return;

    using namespace std::string_view_literals;
    constexpr auto color_str = Color == model::team_color::blue ? "blue"sv : "yellow"sv;

    ssl_protos::simulation::RobotControlResponse res{};
    if (res.ParseFromArray(buf(Color).data(), length)) {
      for (const auto& err : res.errors()) {
        l_.warn(fmt::format("simrator sends error (color: {}, code: {}, msg: {})", color_str,
                            err.has_code() ? err.code() : "<unknown>",
                            err.has_message() ? err.message() : "<unknown>"));
      }

      // TODO: ロボットの情報 (RobotFeedback) を radio::base::target_message に渡す
    } else {
      l_.warn(fmt::format("parse error (color: {})", color_str));
    }

    recv<Color>();
  }

  auto& connection(model::team_color color) {
    return color == model::team_color::blue ? *connection_blue_ : *connection_yellow_;
  }

  auto& buf(model::team_color color) {
    return color == model::team_color::blue ? buf_blue_ : buf_yellow_;
  }

  std::unique_ptr<Connection> connection_blue_;
  std::unique_ptr<Connection> connection_yellow_;

  std::array<char, 4096> buf_blue_;
  std::array<char, 4096> buf_yellow_;

  logger::logger_for<robot> l_;
};

/// ボールやロボット配置などシミュレータ操作の命令を送る (Simulation Control)
template <class Connection>
class simulation : public base::simulator {
public:
  /// @param connection  シミュレータのコマンド送受信に使う connection
  simulation(std::unique_ptr<Connection> connection) : connection_{std::move(connection)} {
    connection_->recv(
        buf_, [this](auto&&... args) { handle_recv(std::forward<decltype(args)>(args)...); });
  }

  void set_ball_position(double x, double y) override {
    ssl_protos::simulation::SimulatorCommand cmd{};
    auto b = cmd.mutable_control()->mutable_teleport_ball();
    b->set_x(x / 1000);
    b->set_y(y / 1000);
    b->set_z(0);
    b->set_vx(0);
    b->set_vy(0);
    b->set_vz(0);
    connection_->send(cmd.SerializeAsString());
  }

  void set_robot_position(model::team_color color, unsigned int id, double x, double y,
                          double theta) override {
    ssl_protos::simulation::SimulatorCommand cmd{};
    auto r = cmd.mutable_control()->add_teleport_robot();
    auto i = r->mutable_id();
    i->set_id(id);
    i->set_team(color == model::team_color::blue ? ssl_protos::gc::Team::BLUE
                                                 : ssl_protos::gc::Team::YELLOW);
    r->set_x(x / 1000);
    r->set_y(y / 1000);
    r->set_orientation(theta);
    r->set_v_x(0);
    r->set_v_y(0);
    r->set_v_angular(0);
    connection_->send(cmd.SerializeAsString());
  }

private:
  void handle_recv(const boost::system::error_code& ec, std::size_t length) {
    if (ec) return;

    ssl_protos::simulation::SimulatorResponse res{};
    if (res.ParseFromArray(buf_.data(), length)) {
      for (const auto& err : res.errors()) {
        l_.warn(fmt::format("simrator sends error (code: {}, msg: {})",
                            err.has_code() ? err.code() : "<unknown>",
                            err.has_message() ? err.message() : "<unknown>"));
      }
    } else {
      l_.warn("parse error (sim)");
    }

    connection_->recv(
        buf_, [this](auto&&... args) { handle_recv(std::forward<decltype(args)>(args)...); });
  }

  std::unique_ptr<Connection> connection_;

  std::array<char, 4096> buf_;
  logger::logger_for<simulation> l_;
};

/// Robot Control Blue/Yellow, Simulation Control 両方に対応した radio
template <class Connection>
class all : public robot<Connection>, public simulation<Connection> {
public:
  /// @param connection_sim     シミュレータのコマンド送受信に使う connection
  /// @param connection_blue    ロボットのコマンド送信・ステータス受信に使う connection (Blue)
  /// @param connection_yellow  ロボットのコマンド送信・ステータス受信に使う connection
  /// (Yellow)
  all(std::unique_ptr<Connection> connection_sim, std::unique_ptr<Connection> connection_blue,
      std::unique_ptr<Connection> connection_yellow)
      : robot<Connection>(std::move(connection_blue), std::move(connection_yellow)),
        simulation<Connection>(std::move(connection_sim)) {}
};

} // namespace ai_server::radio::ssl_simproto

#endif // AI_SERVER_RADIO_SSL_SIMPROTO_H
