#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE world_model_test

#include <stdexcept>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/world.h"

#include "ssl-protos/vision/detection.pb.h"

BOOST_AUTO_TEST_SUITE(world_model)

BOOST_AUTO_TEST_CASE(setter) {
  ai_server::model::world w{};

  ai_server::model::field field{};
  field.set_length(1);
  w.set_field(field);

  ai_server::model::ball ball{123, 456, 789};
  w.set_ball(ball);

  ai_server::model::world::robots_list robots_blue{{1, {1}}, {3, {3}}};
  w.set_robots_blue(robots_blue);

  ai_server::model::world::robots_list robots_yellow{{2, {2}}, {4, {4}}};
  w.set_robots_yellow(robots_yellow);

  BOOST_TEST(w.field().length() == 1);
  BOOST_TEST(w.ball().x() = 123);
  BOOST_TEST(w.robots_blue().size() == 2);
  BOOST_TEST(w.robots_blue().count(1) == 1);
  BOOST_TEST(w.robots_blue().count(3) == 1);
  BOOST_TEST(w.robots_yellow().size() == 2);
  BOOST_TEST(w.robots_yellow().count(2) == 1);
  BOOST_TEST(w.robots_yellow().count(4) == 1);
}

BOOST_AUTO_TEST_SUITE_END()
