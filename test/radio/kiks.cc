#define BOOST_TEST_DYN_LINK

#include <memory>
#include <optional>
#include <string>

#include <boost/test/unit_test.hpp>

#include "ai_server/radio/kiks.h"

namespace model = ai_server::model;
namespace radio = ai_server::radio;

BOOST_AUTO_TEST_SUITE(kiks)

struct mock_connection {
  std::optional<std::vector<std::uint8_t>> last_value;
  void send(std::vector<std::uint8_t> value) {
    last_value = std::move(value);
  }
};

BOOST_AUTO_TEST_CASE(send_command) {
  auto c   = std::make_unique<mock_connection>();
  auto& rc = *c;
  radio::kiks k{std::move(c)};

  BOOST_TEST(!rc.last_value.has_value());

  k.send(model::team_color::yellow, 1, {model::command::kick_type_t::none, 0}, 2, 0, 0, 0);
  {
    BOOST_TEST(rc.last_value.has_value());

    const auto& v = rc.last_value.value();
    BOOST_TEST(v.size() == 11);

    BOOST_TEST(v[0] == 0b00000010);

    BOOST_TEST(v[5] == 0);
    BOOST_TEST(v[6] == 0);

    BOOST_TEST(v[7] == 2 + 3);
    BOOST_TEST(v[8] == 0);

    BOOST_TEST(v[9] == '\r');
    BOOST_TEST(v[10] == '\n');
  }

  rc.last_value.reset();

  k.send(model::team_color::yellow, 1, {model::command::kick_type_t::line, 3}, 2, 0, 0, 0);
  {
    BOOST_TEST(rc.last_value.has_value());

    const auto& v = rc.last_value.value();
    BOOST_TEST(v.size() == 11);

    BOOST_TEST(v[0] == 0b00110010);

    BOOST_TEST(v[5] == 0);
    BOOST_TEST(v[6] == 0);

    BOOST_TEST(v[7] == 2 + 3);
    BOOST_TEST(v[8] == 3);

    BOOST_TEST(v[9] == '\r');
    BOOST_TEST(v[10] == '\n');
  }

  rc.last_value.reset();

  k.send(model::team_color::yellow, 1, {model::command::kick_type_t::chip, 4}, 2, 0, 0, -1);
  {
    BOOST_TEST(rc.last_value.has_value());

    const auto& v = rc.last_value.value();
    BOOST_TEST(v.size() == 11);

    BOOST_TEST(v[0] == 0b10100010);

    BOOST_TEST(v[5] == (1000 & 0xff00) >> 8);
    BOOST_TEST(v[6] == (1000 & 0x00ff));

    BOOST_TEST(v[7] == 2 + 3);
    BOOST_TEST(v[8] == 4);

    BOOST_TEST(v[9] == '\r');
    BOOST_TEST(v[10] == '\n');
  }
}

BOOST_AUTO_TEST_SUITE_END()
