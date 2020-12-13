#define BOOST_TEST_DYN_LINK

#include <boost/test/data/test_case.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/obstacle/field.h"

BOOST_AUTO_TEST_SUITE(field)

namespace model    = ai_server::model;
namespace obstacle = model::obstacle;

BOOST_DATA_TEST_CASE(center_circle_test, boost::unit_test::data::make({0.0, -500.0, 500.0}),
                     margin) {
  const model::field field{};
  const auto o = obstacle::center_circle(field, margin);

  BOOST_TEST(o.geometry.x() == 0.0);
  BOOST_TEST(o.geometry.y() == 0.0);
  BOOST_TEST(o.margin == field.center_radius() + margin);
}

BOOST_DATA_TEST_CASE(enemy_penalty, boost::unit_test::data::make({0.0, -500.0, 500.0}),
                     margin) {
  const model::field field{};
  const auto o = obstacle::enemy_penalty_area(field, margin);

  BOOST_TEST(o.geometry.min_corner().x() == field.front_penalty_x());
  BOOST_TEST(o.geometry.min_corner().y() == field.penalty_y_min());
  // 障害物の領域が十分伸ばされているか
  BOOST_TEST(o.geometry.max_corner().x() > field.back_penalty_x() + 5000.0);
  BOOST_TEST(o.geometry.max_corner().y() == field.penalty_y_max());
  BOOST_TEST(o.margin == margin);
}

BOOST_DATA_TEST_CASE(our_penalty, boost::unit_test::data::make({0.0, -500.0, 500.0}), margin) {
  const model::field field{};
  const auto o = obstacle::our_penalty_area(field, margin);

  // 障害物の領域が十分伸ばされているか
  BOOST_TEST(o.geometry.min_corner().x() < field.x_min() - 5000.0);
  BOOST_TEST(o.geometry.min_corner().y() == field.penalty_y_min());
  BOOST_TEST(o.geometry.max_corner().x() == field.back_penalty_x());
  BOOST_TEST(o.geometry.max_corner().y() == field.penalty_y_max());
  BOOST_TEST(o.margin == margin);
}

BOOST_AUTO_TEST_SUITE_END()
