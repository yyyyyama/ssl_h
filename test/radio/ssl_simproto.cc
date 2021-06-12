#define BOOST_TEST_DYN_LINK

#include <memory>
#include <optional>
#include <string>

#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/radio/ssl_simproto.h"

namespace model = ai_server::model;
namespace radio = ai_server::radio;

BOOST_AUTO_TEST_SUITE(ssl_simproto)

struct mock_connection {
  std::optional<std::string> last_value;
  void send(std::string value) {
    last_value = std::move(value);
  }

  template <class Buffer, class Func>
  void recv([[maybe_unused]] Buffer& buffer, [[maybe_unused]] Func func) {}
};

BOOST_AUTO_TEST_CASE(robot, *boost::unit_test::tolerance(0.0000001)) {
  auto con_blue_ptr   = std::make_unique<mock_connection>();
  auto con_yellow_ptr = std::make_unique<mock_connection>();
  auto& con_blue      = *con_blue_ptr;
  auto& con_yellow    = *con_yellow_ptr;
  radio::ssl_simproto::robot r{std::move(con_blue_ptr), std::move(con_yellow_ptr)};

  BOOST_TEST(!con_blue.last_value.has_value());
  BOOST_TEST(!con_yellow.last_value.has_value());

  r.send(model::team_color::blue, 123, {model::command::kick_type_t::none, 0}, 0, 0, 0, 0);
  {
    // blue の命令は con_blue で送られる
    BOOST_TEST(!con_yellow.last_value.has_value());
    BOOST_TEST(con_blue.last_value.has_value());

    const auto msg = *std::move(con_blue.last_value);
    con_blue.last_value.reset();

    ssl_protos::simulation::RobotControl p{};
    BOOST_TEST(p.ParseFromString(msg));

    BOOST_TEST(p.robot_commands_size() == 1);

    const auto& rc0 = p.robot_commands(0);
    BOOST_TEST(rc0.id() == 123);

    // 移動は local velocity で指示
    BOOST_TEST(rc0.has_move_command());
    const auto& mc = rc0.move_command();
    BOOST_TEST(mc.has_local_velocity());
    const auto& lv = mc.local_velocity();
    BOOST_TEST(lv.forward() == 0);
    BOOST_TEST(lv.left() == 0);
    BOOST_TEST(lv.angular() == 0);

    // ボールを蹴らないときは kick_* を送らない
    BOOST_TEST(!rc0.has_kick_speed());
    BOOST_TEST(!rc0.has_kick_angle());

    // ドリブルしないときは dribbler_speed を送らない
    BOOST_TEST(!rc0.has_dribbler_speed());
  }

  r.send(model::team_color::yellow, 456, {model::command::kick_type_t::line, 10}, 10, 12.3,
         45.6, 7.0);
  {
    // yellow の命令は con_yellow で送られる
    BOOST_TEST(!con_blue.last_value.has_value());
    BOOST_TEST(con_yellow.last_value.has_value());

    const auto msg = *std::move(con_yellow.last_value);
    con_yellow.last_value.reset();

    ssl_protos::simulation::RobotControl p{};
    BOOST_TEST(p.ParseFromString(msg));

    BOOST_TEST(p.robot_commands_size() == 1);

    const auto& rc0 = p.robot_commands(0);
    BOOST_TEST(rc0.id() == 456);

    // 移動は local velocity で指示
    BOOST_TEST(rc0.has_move_command());
    const auto& mc = rc0.move_command();
    BOOST_TEST(mc.has_local_velocity());
    const auto& lv = mc.local_velocity();
    BOOST_TEST(lv.forward() == 12.3 / 1000); // 単位は m/s
    BOOST_TEST(lv.left() == 45.6 / 1000);
    BOOST_TEST(lv.angular() == 7.0);

    // kick_type_t::line -> 角度 0deg
    BOOST_TEST(rc0.has_kick_speed());
    BOOST_TEST(rc0.kick_speed() == ai_server::util::kick::power_to_speed(10) / 1000);
    BOOST_TEST(rc0.has_kick_angle());
    BOOST_TEST(rc0.kick_angle() == 0);

    // ドリブルするときは非ゼロの何らかの値が設定される
    BOOST_TEST(rc0.has_dribbler_speed());
    BOOST_TEST(rc0.dribbler_speed() > 0);
  }

  r.send(model::team_color::blue, 7, {model::command::kick_type_t::chip, 5}, 0, 0, 0, 0);
  {
    // blue の命令は con_blue で送られる
    BOOST_TEST(!con_yellow.last_value.has_value());
    BOOST_TEST(con_blue.last_value.has_value());

    const auto msg = *std::move(con_blue.last_value);
    con_blue.last_value.reset();

    ssl_protos::simulation::RobotControl p{};
    BOOST_TEST(p.ParseFromString(msg));

    BOOST_TEST(p.robot_commands_size() == 1);

    const auto& rc0 = p.robot_commands(0);
    BOOST_TEST(rc0.id() == 7);

    // kick_type_t::chip -> 角度 60deg
    BOOST_TEST(rc0.has_kick_speed());
    BOOST_TEST(rc0.kick_speed() == ai_server::util::kick::power_to_speed(5) / 1000);
    BOOST_TEST(rc0.has_kick_angle());
    BOOST_TEST(rc0.kick_angle() == 60.0);
  }

  r.send(model::team_color::yellow, 8, {model::command::kick_type_t::backspin, 20}, 0, 0, 0, 0);
  {
    // yellow の命令は con_yellow で送られる
    BOOST_TEST(!con_blue.last_value.has_value());
    BOOST_TEST(con_yellow.last_value.has_value());

    const auto msg = *std::move(con_yellow.last_value);
    con_yellow.last_value.reset();

    ssl_protos::simulation::RobotControl p{};
    BOOST_TEST(p.ParseFromString(msg));

    BOOST_TEST(p.robot_commands_size() == 1);

    const auto& rc0 = p.robot_commands(0);
    BOOST_TEST(rc0.id() == 8);

    // kick_type_t::backspin -> 角度 60deg
    BOOST_TEST(rc0.has_kick_speed());
    BOOST_TEST(rc0.kick_speed() == ai_server::util::kick::power_to_speed(20) / 1000);
    BOOST_TEST(rc0.has_kick_angle());
    BOOST_TEST(rc0.kick_angle() == 60.0);
  }
}

BOOST_AUTO_TEST_CASE(simulation, *boost::unit_test::tolerance(0.0000001)) {
  auto con_ptr = std::make_unique<mock_connection>();
  auto& con    = *con_ptr;
  radio::ssl_simproto::simulation r{std::move(con_ptr)};

  BOOST_TEST(!con.last_value.has_value());

  r.set_ball_position(1234, 5678);
  {
    BOOST_TEST(con.last_value.has_value());

    const auto msg = *std::move(con.last_value);
    con.last_value.reset();

    ssl_protos::simulation::SimulatorCommand p{};
    BOOST_TEST(p.ParseFromString(msg));

    BOOST_TEST(p.has_control());

    const auto& ctrl = p.control();
    // ロボット移動メッセージは空
    BOOST_TEST(ctrl.teleport_robot_size() == 0);
    BOOST_TEST(ctrl.has_teleport_ball());

    const auto& b = ctrl.teleport_ball();
    BOOST_TEST(b.x() == 1.234); // 単位は m
    BOOST_TEST(b.y() == 5.678);
    BOOST_TEST(b.z() == 0);
    BOOST_TEST(b.vx() == 0);
    BOOST_TEST(b.vy() == 0);
    BOOST_TEST(b.vz() == 0);
  }

  r.set_robot_position(model::team_color::blue, 123, 456, 789,
                       boost::math::double_constants::half_pi);
  {
    BOOST_TEST(con.last_value.has_value());

    const auto msg = *std::move(con.last_value);
    con.last_value.reset();

    ssl_protos::simulation::SimulatorCommand p{};
    BOOST_TEST(p.ParseFromString(msg));

    BOOST_TEST(p.has_control());

    const auto& ctrl = p.control();
    // ボール移動メッセージは空
    BOOST_TEST(!ctrl.has_teleport_ball());
    BOOST_TEST(ctrl.teleport_robot_size() == 1);

    const auto& r = ctrl.teleport_robot(0);
    BOOST_TEST(r.id().id() == 123);
    BOOST_TEST(r.id().team() == ssl_protos::gc::Team::BLUE);
    BOOST_TEST(r.x() == 0.456);
    BOOST_TEST(r.y() == 0.789);
    BOOST_TEST(r.orientation() == boost::math::double_constants::half_pi);
    BOOST_TEST(r.v_x() == 0);
    BOOST_TEST(r.v_y() == 0);
    BOOST_TEST(r.v_angular() == 0);
  }

  r.set_robot_position(model::team_color::yellow, 456, 78, 90,
                       boost::math::double_constants::pi);
  {
    BOOST_TEST(con.last_value.has_value());

    const auto msg = *std::move(con.last_value);
    con.last_value.reset();

    ssl_protos::simulation::SimulatorCommand p{};
    BOOST_TEST(p.ParseFromString(msg));

    BOOST_TEST(p.has_control());

    const auto& ctrl = p.control();
    // ボール移動メッセージは空
    BOOST_TEST(!ctrl.has_teleport_ball());
    BOOST_TEST(ctrl.teleport_robot_size() == 1);

    const auto& r = ctrl.teleport_robot(0);
    BOOST_TEST(r.id().id() == 456);
    BOOST_TEST(r.id().team() == ssl_protos::gc::Team::YELLOW);
    BOOST_TEST(r.x() == 0.078);
    BOOST_TEST(r.y() == 0.090);
    BOOST_TEST(r.orientation() == boost::math::double_constants::pi);
    BOOST_TEST(r.v_x() == 0);
    BOOST_TEST(r.v_y() == 0);
    BOOST_TEST(r.v_angular() == 0);
  }
}

BOOST_AUTO_TEST_CASE(all, *boost::unit_test::tolerance(0.0000001)) {
  auto con_sim_ptr    = std::make_unique<mock_connection>();
  auto con_blue_ptr   = std::make_unique<mock_connection>();
  auto con_yellow_ptr = std::make_unique<mock_connection>();
  auto& con_sim       = *con_sim_ptr;
  auto& con_blue      = *con_blue_ptr;
  auto& con_yellow    = *con_yellow_ptr;
  radio::ssl_simproto::all r{std::move(con_sim_ptr), std::move(con_blue_ptr),
                             std::move(con_yellow_ptr)};

  r.send(model::team_color::blue, 123, {model::command::kick_type_t::none, 0}, 0, 0, 0, 0);
  {
    // blue の命令は con_blue で送られる
    BOOST_TEST(!con_sim.last_value.has_value());
    BOOST_TEST(!con_yellow.last_value.has_value());
    BOOST_TEST(con_blue.last_value.has_value());

    con_blue.last_value.reset();
  }

  r.send(model::team_color::yellow, 123, {model::command::kick_type_t::none, 0}, 0, 0, 0, 0);
  {
    // yellow の命令は con_yellow で送られる
    BOOST_TEST(!con_sim.last_value.has_value());
    BOOST_TEST(!con_blue.last_value.has_value());
    BOOST_TEST(con_yellow.last_value.has_value());

    con_yellow.last_value.reset();
  }

  r.set_ball_position(1234, 5678);
  {
    // simulator の命令は con_sim で送られる
    BOOST_TEST(!con_blue.last_value.has_value());
    BOOST_TEST(!con_yellow.last_value.has_value());
    BOOST_TEST(con_sim.last_value.has_value());

    con_sim.last_value.reset();
  }
}

BOOST_AUTO_TEST_SUITE_END()
