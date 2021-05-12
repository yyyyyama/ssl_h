#define BOOST_TEST_DYN_LINK

#include <memory>
#include <optional>
#include <string>

#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/radio/grsim.h"

#include "ssl-protos/grsim_packet.pb.h"

namespace model = ai_server::model;
namespace radio = ai_server::radio;

BOOST_AUTO_TEST_SUITE(grsim)

struct mock_connection {
  std::optional<std::string> last_value;
  void send(std::string value) {
    last_value = std::move(value);
  }
};

struct wrapper : public radio::grsim<mock_connection> {
  static ssl_protos::grsim::Command convert2(unsigned int id,
                                             const model::command::kick_flag_t& kick_flag,
                                             int dribble, double vx, double vy, double omega) {
    ssl_protos::grsim::Command c{};
    convert(id, kick_flag, dribble, vx, vy, omega, c);
    return c;
  }
};

BOOST_AUTO_TEST_CASE(to_gr_command) {
  {
    const auto c = wrapper::convert2(123, {model::command::kick_type_t::none, 0}, 456, 4, 8, 9);

    BOOST_TEST(c.id() == 123);

    BOOST_TEST(c.kickspeedx() == 0);
    BOOST_TEST(c.kickspeedz() == 0);

    BOOST_TEST(c.veltangent() = 7.0 / 1000);
    BOOST_TEST(c.velnormal() = 8.0 / 1000);
    BOOST_TEST(c.velangular() = 9.0);

    BOOST_TEST(c.spinner());
    BOOST_TEST(!c.wheelsspeed());
  }

  {
    const auto c = wrapper::convert2(123, {model::command::kick_type_t::none, 0}, 0, 4, 8, 9);
    BOOST_TEST(!c.spinner());
  }

  {
    const auto c = wrapper::convert2(123, {model::command::kick_type_t::line, 321}, 0, 4, 8, 9);
    BOOST_TEST(c.kickspeedx() == ai_server::util::kick::power_to_speed(321) / 1000);
    BOOST_TEST(c.kickspeedz() == 0);
  }

  {
    const auto c = wrapper::convert2(123, {model::command::kick_type_t::chip, 654}, 0, 4, 8, 9);
    BOOST_TEST(c.kickspeedx() == 0.5 * ai_server::util::kick::power_to_speed(654) / 1000);
    BOOST_TEST(c.kickspeedz() ==
               0.5 * std::pow(3, 1 / 2) * ai_server::util::kick::power_to_speed(654) / 1000);
  }

  {
    const auto c =
        wrapper::convert2(123, {model::command::kick_type_t::backspin, 987}, 0, 4, 8, 9);
    BOOST_TEST(c.kickspeedx() == 0.5 * ai_server::util::kick::power_to_speed(987) / 1000);
    BOOST_TEST(c.kickspeedz() ==
               0.5 * std::pow(3, 1 / 2) * ai_server::util::kick::power_to_speed(987) / 1000);
  }
}

BOOST_AUTO_TEST_CASE(send_command) {
  auto c   = std::make_unique<mock_connection>();
  auto& rc = *c;
  radio::grsim g{std::move(c)};

  BOOST_TEST(!rc.last_value.has_value());

  g.send(model::team_color::yellow, 123, {model::command::kick_type_t::none, 0}, 0, 0, 0, 0);
  {
    BOOST_TEST(rc.last_value.has_value());

    ssl_protos::grsim::Packet p{};
    BOOST_TEST(p.ParseFromString(rc.last_value.value()));

    BOOST_TEST(p.has_commands());
    BOOST_TEST(!p.has_replacement());

    const auto& pc = p.commands();
    BOOST_TEST(pc.isteamyellow());

    const auto& pcc = pc.robot_commands();
    BOOST_TEST(pcc.size() == 1);
    BOOST_TEST(pcc[0].id() == 123);
  }

  rc.last_value.reset();

  g.send(model::team_color::blue, 456, {model::command::kick_type_t::none, 0}, 0, 0, 0, 0);
  {
    BOOST_TEST(rc.last_value.has_value());

    ssl_protos::grsim::Packet p{};
    BOOST_TEST(p.ParseFromString(rc.last_value.value()));

    BOOST_TEST(p.has_commands());
    BOOST_TEST(!p.has_replacement());

    const auto& pc = p.commands();
    BOOST_TEST(!pc.isteamyellow());

    const auto& pcc = pc.robot_commands();
    BOOST_TEST(pcc.size() == 1);
    BOOST_TEST(pcc[0].id() == 456);
  }
}

BOOST_AUTO_TEST_CASE(replacement_ball) {
  auto c   = std::make_unique<mock_connection>();
  auto& rc = *c;
  radio::grsim g{std::move(c)};

  BOOST_TEST(!rc.last_value.has_value());

  g.set_ball_position(1234, 5678);
  {
    BOOST_TEST(rc.last_value.has_value());

    ssl_protos::grsim::Packet p{};
    BOOST_TEST(p.ParseFromString(rc.last_value.value()));

    BOOST_TEST(!p.has_commands());
    BOOST_TEST(p.has_replacement());

    const auto& pr = p.replacement();
    BOOST_TEST(pr.has_ball());

    const auto& b = pr.ball();
    BOOST_TEST(b.x() == 1.234); // grSim は座標を m で扱う
    BOOST_TEST(b.y() == 5.678);
    BOOST_TEST(b.vx() == 0);
    BOOST_TEST(b.vy() == 0);
  }
}

BOOST_AUTO_TEST_CASE(replacement_robot) {
  auto c   = std::make_unique<mock_connection>();
  auto& rc = *c;
  radio::grsim g{std::move(c)};

  BOOST_TEST(!rc.last_value.has_value());

  g.set_robot_position(model::team_color::yellow, 123, 456, 789,
                       boost::math::double_constants::half_pi);
  {
    BOOST_TEST(rc.last_value.has_value());

    ssl_protos::grsim::Packet p{};
    BOOST_TEST(p.ParseFromString(rc.last_value.value()));

    BOOST_TEST(!p.has_commands());
    BOOST_TEST(p.has_replacement());

    const auto& pr = p.replacement();

    const auto& r = pr.robots();
    BOOST_TEST(r.size() == 1);
    BOOST_TEST(r[0].id() == 123);
    BOOST_TEST(r[0].x() == 0.456);
    BOOST_TEST(r[0].y() == 0.789);
    BOOST_TEST(r[0].dir() == 90); // grSim は角度を deg で扱う
    BOOST_TEST(r[0].yellowteam());
  }

  rc.last_value.reset();

  g.set_robot_position(model::team_color::blue, 456, 78, 90, boost::math::double_constants::pi);
  {
    BOOST_TEST(rc.last_value.has_value());

    ssl_protos::grsim::Packet p{};
    BOOST_TEST(p.ParseFromString(rc.last_value.value()));

    BOOST_TEST(!p.has_commands());
    BOOST_TEST(p.has_replacement());

    const auto& pr = p.replacement();

    const auto& r = pr.robots();
    BOOST_TEST(r.size() == 1);
    BOOST_TEST(r[0].id() == 456);
    BOOST_TEST(r[0].x() == 0.078);
    BOOST_TEST(r[0].y() == 0.09);
    BOOST_TEST(r[0].dir() == 180);
    BOOST_TEST(!r[0].yellowteam());
  }
}

BOOST_AUTO_TEST_SUITE_END()
